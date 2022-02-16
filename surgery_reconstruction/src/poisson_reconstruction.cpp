#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <open3d/Open3D.h>
#include <open3d_conversions/open3d_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

void Open3DMeshToROS(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh,
                     visualization_msgs::Marker &triangle_list_marker, int id,
                     std_msgs::Header header, geometry_msgs::Pose pose,
                     geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color) {
  int m = mesh->triangles_.size();
  std::vector<geometry_msgs::Point> points(m * 3);
  for (int i = 0; i < m; ++i) {
    auto triangle = mesh->triangles_[i];
    for (int j = 0; j < 3; ++j) {
      auto v = mesh->vertices_[triangle[j]];
      points[3 * i + j].x = v[0];
      points[3 * i + j].y = v[1];
      points[3 * i + j].z = v[2];
    }
  }
  triangle_list_marker.header = header;
  triangle_list_marker.ns = "o3d";
  triangle_list_marker.id = id;
  triangle_list_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  triangle_list_marker.action = visualization_msgs::Marker::ADD;
  triangle_list_marker.pose = pose;
  triangle_list_marker.scale = scale;
  triangle_list_marker.color = color;
  triangle_list_marker.points = points;
}

class PoissonReconstructionNode {
  std::string const cloud_topic = "static_registration/aligned_points";
  std::string const mesh_topic = "poisson/reconstruction";

public:
  PoissonReconstructionNode(ros::NodeHandle &nh) : nh_(nh) {
    cloud_sub_ = nh_.subscribe(
        cloud_topic, 1, &PoissonReconstructionNode::pointCloud2Callback, this);
    mesh_pub_ = nh_.advertise<visualization_msgs::Marker>(mesh_topic, 1);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher mesh_pub_;

  void pointCloud2Callback(sensor_msgs::PointCloud2::ConstPtr const &cloud) {
    visualization_msgs::Marker triangle_list_marker;
    createMeshTriangleList(cloud, triangle_list_marker);
    mesh_pub_.publish(triangle_list_marker);
  }

  void
  createMeshTriangleList(sensor_msgs::PointCloud2::ConstPtr const &cloud,
                         visualization_msgs::Marker &triangle_list_marker) {

    open3d::geometry::PointCloud o3d_cloud;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    std::vector<double> densities;
    ROS_INFO("Converting ROS PC2 to open3d");
    open3d_conversions::rosToOpen3d(cloud, o3d_cloud);

    o3d_cloud.RemoveNonFinitePoints();

    if (not o3d_cloud.HasNormals()) {
      ROS_INFO("Normals Estimation");
      o3d_cloud.EstimateNormals();
      ROS_INFO("Normals Orientation");
      o3d_cloud.OrientNormalsTowardsCameraLocation();
    }

    ROS_INFO("Computing Mesh");

    open3d::utility::VerbosityContextManager cm(
        open3d::utility::VerbosityLevel::Debug);
    cm.Enter();
    std::tie(mesh, densities) =
        open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
            o3d_cloud, 5, 0, 1.1f, false, 8);
    cm.Exit();

    ROS_INFO("COMPUTED MESH");
    /* Setting Message attributes */
    std_msgs::Header header;
    header.stamp = cloud->header.stamp;
    header.frame_id = cloud->header.frame_id;

    geometry_msgs::Point position;
    position.x = 0.0, position.y = 0.0, position.z = 0.0;

    geometry_msgs::Quaternion orientation;
    orientation.x = 0.0, orientation.y = 0.0, orientation.z = 0.0,
    orientation.w = 1.0;

    geometry_msgs::Pose pose;
    pose.position = position, pose.orientation = orientation;

    geometry_msgs::Vector3 scale;
    scale.x = 1.0, scale.y = 1.0, scale.z = 1.0;

    std_msgs::ColorRGBA color;
    color.r = 0.0, color.g = 1.0, color.b = 0.0, color.a = 0.8;

    ROS_INFO("Building Message");
    Open3DMeshToROS(mesh, triangle_list_marker, 0, header, pose, scale, color);
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "poisson_reconstruction");
  ros::NodeHandle nh;
  PoissonReconstructionNode rn(nh);
  ros::spin();
  return 0;
}
