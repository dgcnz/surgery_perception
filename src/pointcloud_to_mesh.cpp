#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <memory>
#include <mesh_conversions/conversions.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <utility>

class ReconstructionNode
{
    using Vec       = lvr2::BaseVector<float>;
    using PsSurface = lvr2::PointsetSurface<Vec>;

  public:
    ReconstructionNode(ros::NodeHandle &nh);

  private:
    ros::NodeHandle    nh_;
    ros::Subscriber    cloud_sub_;
    ros::Publisher     mesh_pub_;
    ros::ServiceServer service_;

    ReconstructionNode(ros::NodeHandle &nh) : nh_(nh) {}

    void pointCloud2Callback(sensor_msgs::PointCloud2::ConstPtr const &cloud)
    {
        mesh_msgs::MeshGeometryStamped mesh_msg;
        generateMeshGeometry(cloud, mesh_msg);
        mesh_pub_.publish(mesh_msg);
    }

    bool
    createMeshGeometryMessage(sensor_msgs::PointCloud2 const &      cloud,
                              mesh_msgs::MeshGeometryStamped const &mesh_msg)
    {
        lvr2::MeshBufferPtr  mesh_buffer_ptr(new lvr2::MeshBuffer);
        lvr2::PointBufferPtr point_buffer_ptr(new lvr2::PointBuffer);

        if (!mesh_msgs::fromPointCloud2ToPointBuffer(cloud, *point_buffer_ptr))
        {
            ROS_ERROR_STREAM("Could not convert point cloud from "
                             "sensor_msgs::PointCloud2 to "
                             "lvr2::PointBuffer!");
            return false;
        }
        if (!createMeshGeometry(point_buffer_ptr, mesh_buffer_ptr))
        {
            ROS_ERROR_STREAM("Reconstruction failed!");
            return false;
        }

        lvr2::HalfEdgeMesh<Vec> hem;
        if (!createMeshGeometry(point_buffer_ptr, hem))
        {
            ROS_ERROR_STREAM(
                "Could not convert point cloud from lvr2::MeshBuffer to "
                "lvr::HalfEdgeMesh");
            return false;
        }

        mesh_msg = mesh_msgs::toMeshGeometryStamped(
            hem, cloud.header.frame_id, to_string(cloud.header.seq));
        return true;
    }

    std::pair<std::shared_ptr<GridBase>,
              std::unique_ptr<FastReconstructionBase<Vec>>>
    createGridAndReconstruction(PointsetSurfacePtr<Vec> surface)
    {
        int   intersections = -1;
        float voxelsize     = 10;

        bool   useVoxelSize      = intersections <= 0;
        float  resolutaion       = useVoxelSize ? voxelsize : intersections;
        string decompositionType = "PMC";

        BilinearFastBox<Vec>::m_surface = surface;
        auto grid = std::make_shared<PointsetGrid<Vec, BilinearFastBox<Vec>>>(
            resolution,
            surface,
            surface->getBoundingBox(),
            useVoxelSize,
            false);
        grid->calcDistanceValues();
        auto reconstruction =
            make_unique<FastReconstruction<Vec, BilinearFastBox<Vec>>>(grid);
        return make_pair(grid, std::move(reconstruction));
    }

    bool createMeshGeometry(lvr2::PointBufferPtr &   point_buffer_ptr,
                            lvr2::HalfEdgeMesh<Vec> &mesh)
    {
        auto surface = createSurface(point_buffer_ptr);
        if (!surface)
        {
            ROS_ERROR_STREAM("Failed to create surface from point_buffer_ptr!");
            return false;
        }
        auto [grid, reconstruction] = createGridAndReconstruction(surface);
        reconstruction->getMesh(mesh);
        return true;
    }

    PointsetSurfacePtr<Vec>
    createSurface(lvr2::PointBufferPtr &point_buffer_ptr)
    {
        string pcm_name         = "FLANN"; // default value
        int    plane_fit_method = 0;
        int    kd               = 5;
        int    ki               = 10;
        int    kn               = 10;

        PointsetSurfacePtr<Vec> surface =
            make_shared<AdaptiveKSearchSurface<Vec>>(
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
