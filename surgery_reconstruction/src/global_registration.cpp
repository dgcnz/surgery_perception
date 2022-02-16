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

class GlobalRegistrationNode {
  using MySyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                      sensor_msgs::PointCloud2>;

  std::string const in_cloud1_topic = "/camera_01/depth_registered/points";
  std::string const in_cloud2_topic = "/camera_02/depth_registered/points";
  std::string const out_cloud_topic = "/global_registration/aligned_points";
  std::string const out_tf2_topic = "/global_registration/tf2";
  ros::NodeHandle nh;
  ros::Publisher cloud_pub;
  tf2_ros::TransformBroadcaster tf2_pub;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub;

public:
  GlobalRegistrationNode(ros::NodeHandle &nh_) : nh(nh_) {
    cloud1_sub.subscribe(nh, in_cloud1_topic, 1);
    cloud2_sub.subscribe(nh, in_cloud2_topic, 1);
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(
        MySyncPolicy(10), cloud1_sub, cloud2_sub));
    sync->registerCallback(
        boost::bind(&GlobalRegistrationNode::callback, this, _1, _2));
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 1);
  }

  void callback(sensor_msgs::PointCloud2::ConstPtr const &cloud1,
                sensor_msgs::PointCloud2::ConstPtr const &cloud2) {
    // transform to o3d pc
    //
    ROS_INFO("Converting sensor_msgs::PointCloud2 to Open3D::PointCloud");
    open3d::geometry::PointCloud o3d_cloud1;
    open3d_conversions::rosToOpen3d(cloud1, o3d_cloud1);
    open3d::geometry::PointCloud o3d_cloud2;
    open3d_conversions::rosToOpen3d(cloud2, o3d_cloud2);

    auto registration_result = compute_transformation(o3d_cloud1, o3d_cloud2);

    ROS_INFO("Merging PointClouds");
    // Publish aligned cloud (optional)
    o3d_cloud1.Transform(registration_result.transformation_);
    o3d_cloud1 += o3d_cloud2;

    sensor_msgs::PointCloud2 out_cloud;
    open3d_conversions::open3dToRos(o3d_cloud1, out_cloud);
    ROS_INFO("Publishing PointClouds");
    cloud_pub.publish(out_cloud);

    // Publish tf2 (TODO: make static)
    Eigen::Isometry3d tf_matrix;
    tf_matrix.matrix() = registration_result.transformation_;
    geometry_msgs::TransformStamped tf2_msg = tf2::eigenToTransform(tf_matrix);
    tf2_msg.header.frame_id = cloud1->header.frame_id;
    tf2_msg.child_frame_id = cloud2->header.frame_id;
    ROS_INFO("Publishing tf2 transform");
    tf2_pub.sendTransform(tf2_msg);
  }

  std::pair<std::shared_ptr<open3d::geometry::PointCloud>,
            std::shared_ptr<open3d::pipelines::registration::Feature>>
  preprocess_pointcloud(open3d::geometry::PointCloud &cloud,
                        double voxel_size) {

    // TODO: radius constants?
    using open3d::geometry::KDTreeSearchParamHybrid;
    using open3d::geometry::PointCloud;
    using open3d::pipelines::registration::ComputeFPFHFeature;
    using open3d::pipelines::registration::Feature;

    cloud.RemoveNonFinitePoints();
    std::shared_ptr<PointCloud> cloud_down = cloud.VoxelDownSample(voxel_size);

    double radius_normal = voxel_size * 2;
    if (not cloud_down->HasNormals()) {
      cloud_down->EstimateNormals(KDTreeSearchParamHybrid(radius_normal, 30));
      cloud_down->OrientNormalsTowardsCameraLocation();
    }

    double radius_feature = voxel_size * 5;
    std::shared_ptr<Feature> cloud_fpfh = ComputeFPFHFeature(
        *cloud_down, KDTreeSearchParamHybrid(radius_feature, 100));
    return std::make_pair(cloud_down, cloud_fpfh);
  }

  open3d::pipelines::registration::RegistrationResult
  compute_transformation(open3d::geometry::PointCloud &cloud1,
                         open3d::geometry::PointCloud &cloud2) {

    using open3d::pipelines::registration::CorrespondenceChecker;
    using open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance;
    using open3d::pipelines::registration::
        CorrespondenceCheckerBasedOnEdgeLength;
    using open3d::pipelines::registration::RANSACConvergenceCriteria;
    using open3d::pipelines::registration::
        RegistrationRANSACBasedOnFeatureMatching;
    using open3d::pipelines::registration::
        TransformationEstimationForColoredICP;
    using open3d::pipelines::registration::TransformationEstimationPointToPoint;

    double voxel_size = 0.05; // 5cm
    double distance_threshold = voxel_size * 1.5;

    ROS_INFO("Preprocessing clouds");
    auto [cloud1_down, cloud1_fpfh] = preprocess_pointcloud(cloud1, voxel_size);
    auto [cloud2_down, cloud2_fpfh] = preprocess_pointcloud(cloud2, voxel_size);

    auto ccb_edge_length = CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto ccb_distance =
        CorrespondenceCheckerBasedOnDistance(distance_threshold);

    std::vector<std::reference_wrapper<const CorrespondenceChecker>> checkers =
        {std::ref(ccb_edge_length), std::ref(ccb_distance)};

    ROS_INFO("Computing Global Registration");
    auto trans_estimation = TransformationEstimationPointToPoint(false);
    // auto trans_estimation = TransformationEstimationForColoredICP();
    return RegistrationRANSACBasedOnFeatureMatching(
        *cloud1_down, *cloud2_down, *cloud1_fpfh, *cloud2_fpfh, true,
        distance_threshold, trans_estimation, 3, checkers,
        RANSACConvergenceCriteria(100000, 0.999));
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "global_registration");
  ros::NodeHandle nh;
  ROS_INFO("Creating GlobalRegistrationNode");
  GlobalRegistrationNode rn(nh);
  ros::spin();
  return 0;
}
