#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <open3d/Open3D.h>
#include <open3d_conversions/open3d_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

class StaticRegistrationNode {
  using MySyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                      sensor_msgs::PointCloud2>;

  std::string const in_cloud1_topic = "/camera_01/depth_registered/points";
  std::string const in_cloud2_topic = "/camera_02/depth_registered/points";
  std::string const out_cloud_topic = "/static_registration/aligned_points";
  std::string const out_tf2_topic = "/static_registration/tf2";
  ros::NodeHandle nh;
  ros::Publisher cloud_pub;
  tf2_ros::TransformBroadcaster tf2_pub;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub;
  Eigen::Matrix<double, 4, 4> transform;

public:
  StaticRegistrationNode(ros::NodeHandle &nh_) : nh(nh_) {
    cloud1_sub.subscribe(nh, in_cloud1_topic, 1);
    cloud2_sub.subscribe(nh, in_cloud2_topic, 1);
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(
        MySyncPolicy(10), cloud1_sub, cloud2_sub));
    sync->registerCallback(
        boost::bind(&StaticRegistrationNode::callback, this, _1, _2));
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 1);
    transform << 0.67541945, 0.072289, -0.73388205, 0.66049106, -0.11761348,
        0.99300466, -0.01043099, -0.0101487, 0.72799426, 0.09335971, 0.67919682,
        0.25830009, 0.0, 0.0, 0.0, 1.0;
  }

  void callback(sensor_msgs::PointCloud2::ConstPtr const &cloud1,
                sensor_msgs::PointCloud2::ConstPtr const &cloud2) {
    // transform to o3d pc
    ROS_INFO("Converting sensor_msgs::PointCloud2 to Open3D::PointCloud");
    open3d::geometry::PointCloud o3d_cloud1;
    open3d_conversions::rosToOpen3d(cloud1, o3d_cloud1);
    open3d::geometry::PointCloud o3d_cloud2;
    open3d_conversions::rosToOpen3d(cloud2, o3d_cloud2);

    preprocess_pointcloud(o3d_cloud1);
    preprocess_pointcloud(o3d_cloud2);
    ROS_INFO("Merging PointClouds");
    // Publish aligned cloud (optional)
    o3d_cloud1.Transform(transform);
    o3d_cloud1 += o3d_cloud2;

    sensor_msgs::PointCloud2 out_cloud;
    open3d_conversions::open3dToRos(o3d_cloud1, out_cloud);
    ROS_INFO("Publishing PointClouds");
    cloud_pub.publish(out_cloud);

    // Publish tf2 (TODO: make static)
    Eigen::Isometry3d tf_matrix;
    tf_matrix.matrix() = transform;
    geometry_msgs::TransformStamped tf2_msg = tf2::eigenToTransform(tf_matrix);
    tf2_msg.header.frame_id = cloud1->header.frame_id;
    tf2_msg.child_frame_id = cloud2->header.frame_id;
    ROS_INFO("Publishing tf2 transform");
    tf2_pub.sendTransform(tf2_msg);
  }

  void preprocess_pointcloud(open3d::geometry::PointCloud &cloud) {
    cloud.RemoveNonFinitePoints();
    if (not cloud.HasNormals()) {
      cloud.EstimateNormals();
      cloud.OrientNormalsTowardsCameraLocation();
    }
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "static_registration");
  ros::NodeHandle nh;
  ROS_INFO("Creating StaticRegistrationNode");
  StaticRegistrationNode rn(nh);
  ros::spin();
  return 0;
}
