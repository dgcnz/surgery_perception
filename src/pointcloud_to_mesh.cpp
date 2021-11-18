#include <lvr2/algorithm/CleanupAlgorithms.hpp>
#include <lvr2/algorithm/ClusterAlgorithms.hpp>
#include <lvr2/algorithm/Materializer.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <lvr2/algorithm/Texturizer.hpp>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/geometry/Handles.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/io/PointBuffer.hpp>
#include <lvr2/reconstruction/AdaptiveKSearchSurface.hpp>
#include <lvr2/reconstruction/BilinearFastBox.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/PointsetGrid.hpp>
#include <lvr2/reconstruction/PointsetSurface.hpp>
#include <lvr2/util/ClusterBiMap.hpp>
#include <memory>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs_conversions/conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <utility>
#include <vector>

class ReconstructionNode
{
    using Vec                     = lvr2::BaseVector<float>;
    using PsSurface               = lvr2::PointsetSurface<Vec>;
    std::string const cloud_topic = "camera/depth_registered/points";
    std::string const mesh_topic  = "lvr2/reconstruction";

  public:
    ReconstructionNode(ros::NodeHandle &nh) : nh_(nh)
    {
        cloud_sub_ = nh_.subscribe(
            cloud_topic, 20, &ReconstructionNode::pointCloud2Callback, this);
        mesh_pub_ =
            nh_.advertise<mesh_msgs::MeshGeometryStamped>(mesh_topic, 1);
    }

  private:
    ros::NodeHandle    nh_;
    ros::Subscriber    cloud_sub_;
    ros::Publisher     mesh_pub_;
    ros::ServiceServer service_;

    void pointCloud2Callback(sensor_msgs::PointCloud2::ConstPtr const &cloud)
    {
        mesh_msgs::MeshGeometryStamped mesh_msg;
        createMeshGeometryMessage(*cloud, mesh_msg);
        mesh_pub_.publish(mesh_msg);
    }

    bool createMeshGeometryMessage(sensor_msgs::PointCloud2 const &cloud,
                                   mesh_msgs::MeshGeometryStamped &mesh_msg)
    {
        lvr2::MeshBufferPtr  mesh_buffer_ptr(new lvr2::MeshBuffer);
        lvr2::PointBufferPtr point_buffer_ptr(new lvr2::PointBuffer);

        if (!mesh_msgs_conversions::fromPointCloud2ToPointBuffer(
                cloud, *point_buffer_ptr))
        {
            ROS_ERROR_STREAM("Could not convert point cloud from "
                             "sensor_msgs::PointCloud2 to "
                             "lvr2::PointBuffer!");
            return false;
        }
        if (!createMeshGeometry(point_buffer_ptr, mesh_buffer_ptr))
        {
            ROS_ERROR_STREAM("Reconstruction Failed!");
            return false;
        }
        if (!mesh_msgs_conversions::fromMeshBufferToMeshGeometryMessage(
                mesh_buffer_ptr, mesh_msg.mesh_geometry))
        {

            ROS_ERROR_STREAM("Could not convert point cloud from mesh_buffer "
                             "to Geometry Message!");
            return false;
        }
        mesh_msg.header.frame_id = cloud.header.frame_id;
        mesh_msg.header.stamp    = cloud.header.stamp;
        mesh_msg.uuid            = std::to_string(cloud.header.seq);
        return true;
    }

    std::pair<std::shared_ptr<lvr2::GridBase>,
              std::unique_ptr<lvr2::FastReconstructionBase<Vec>>>
    createGridAndReconstruction(lvr2::PointsetSurfacePtr<Vec> surface)
    {
        int   intersections = -1;
        float voxelsize     = 10;

        bool   useVoxelSize      = intersections <= 0;
        float  resolution        = useVoxelSize ? voxelsize : intersections;
        string decompositionType = "PMC";

        lvr2::BilinearFastBox<Vec>::m_surface = surface;
        auto grid                             = std::make_shared<
            lvr2::PointsetGrid<Vec, lvr2::BilinearFastBox<Vec>>>(
            resolution,
            surface,
            surface->getBoundingBox(),
            useVoxelSize,
            false);
        grid->calcDistanceValues();
        auto reconstruction = make_unique<
            lvr2::FastReconstruction<Vec, lvr2::BilinearFastBox<Vec>>>(grid);
        return make_pair(grid, std::move(reconstruction));
    }

    bool createMeshGeometry(lvr2::PointBufferPtr &point_buffer_ptr,
                            lvr2::MeshBufferPtr & mesh_buffer)
    {
        auto surface = createSurface(point_buffer_ptr);
        if (!surface)
        {
            ROS_ERROR_STREAM("Failed to create surface from point_buffer_ptr!");
            return false;
        }
        auto [grid, reconstruction] = createGridAndReconstruction(surface);
        lvr2::HalfEdgeMesh<Vec> mesh;
        reconstruction->getMesh(mesh);
        int cleanContourIterations = 0;
        lvr2::cleanContours(mesh, cleanContourIterations, 0.0001);
        auto faceNormals = lvr2::calcFaceNormals(mesh);

        float                                normal_threshold = 0.85;
        lvr2::ClusterBiMap<lvr2::FaceHandle> clusterBiMap =
            lvr2::planarClusterGrowing(mesh, faceNormals, normal_threshold);

        // TODO
        lvr2::ClusterPainter painter(clusterBiMap);
        auto                 clusterColors =
            boost::optional<lvr2::DenseClusterMap<lvr2::Rgb8Color>>(
                painter.simpsons(mesh));
        auto vertexColors = lvr2::calcColorFromPointCloud(mesh, surface);
        auto vertexNormals =
            lvr2::calcVertexNormals(mesh, faceNormals, *surface);
        lvr2::TextureFinalizer<Vec> finalize(clusterBiMap);
        finalize.setVertexNormals(vertexNormals);
        lvr2::Materializer<Vec> materializer(
            mesh, clusterBiMap, faceNormals, *surface);
        int texel_size           = 1;
        int tex_min_cluster_size = 100;
        int tex_max_cluster_size = 0;

        lvr2::Texturizer<Vec> texturizer(
            texel_size, tex_min_cluster_size, tex_max_cluster_size);

        lvr2::MaterializerResult<Vec> matResult =
            materializer.generateMaterials();
        finalize.setMaterializerResult(matResult);
        mesh_buffer = finalize.apply(mesh);
        return true;
    }

    lvr2::PointsetSurfacePtr<Vec>
    createSurface(lvr2::PointBufferPtr &point_buffer_ptr)
    {
        string pcm_name         = "FLANN"; // default value
        int    plane_fit_method = 0;
        int    kd               = 5;
        int    ki               = 10;
        int    kn               = 10;

        lvr2::PointsetSurfacePtr<Vec> surface =
            make_shared<lvr2::AdaptiveKSearchSurface<Vec>>(
                point_buffer_ptr, pcm_name, kn, ki, kd, plane_fit_method);

        surface->setKd(kd);
        surface->setKn(kn);
        surface->setKi(ki);
        // check lvr2_reconstruct/Main.cpp for gpu usage
        surface->calculateSurfaceNormals();
        return surface;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reconstruction_node");
    ros::NodeHandle    nh;
    ReconstructionNode rn(nh);
    ros::spin();
    return 0;
}
