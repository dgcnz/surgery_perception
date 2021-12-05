#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <open3d/Open3D.h>
#include <open3d_conversions/open3d_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

class GlobalRegistrationNode {
  std::string const in_cloud1_topic = "camera1/depth_registered/points";
  std::string const in_cloud2_topic = "camera2/depth_registered/points";
  std::string const out_cloud_topic = "global_registration/aligned_points";
  std::string const out_tf2_topic = "global_registration/tf2";
  ros::NodeHandle nh;
  ros::Publisher cloud_pub;
  ros::Publisher tf2_pub;

public:
  GlobalRegistrationNode(ros::NodeHandle &nh_) : nh(nh_) {
    using message_filters::Subscriber;
    using message_filters::TimeSynchronizer;
    using sensor_msgs::PointCloud2;
    Subscriber<PointCloud2> cloud1_sub(nh, in_cloud1_topic, 1);
    Subscriber<PointCloud2> cloud2_sub(nh, in_cloud2_topic, 1);
    TimeSynchronizer<PointCloud2, PointCloud2> sync(cloud1_sub, cloud2_sub, 10);
    sync.registerCallback(
        boost::bind(&GlobalRegistrationNode::callback, this, _1, _2));
    cloud_pub = nh_.advertise<PointCloud2>(out_cloud_topic, 1);
    tf2_pub = nh_.advertise<geometry_msgs::TransformStamped>(out_tf2_topic, 1);
  }

  void callback(sensor_msgs::PointCloud2::ConstPtr const &cloud1,
                sensor_msgs::PointCloud2::ConstPtr const &cloud2) {
    // transform to o3d pc
    open3d::geometry::PointCloud o3d_cloud1;
    open3d_conversions::rosToOpen3d(cloud1, o3d_cloud1);
    open3d::geometry::PointCloud o3d_cloud2;
    open3d_conversions::rosToOpen3d(cloud2, o3d_cloud2);

    auto registration_result = compute_transformation(o3d_cloud1, o3d_cloud2);

    // Publish aligned cloud (optional)
    o3d_cloud1.Transform(registration_result.transformation_);
    o3d_cloud1 += o3d_cloud2;

    sensor_msgs::PointCloud2 out_cloud;
    open3d_conversions::open3dToRos(o3d_cloud1, out_cloud);
    cloud_pub.publish(out_cloud);

    // Publish tf2 (TODO: make static)
    Eigen::Affine3d at;
    at.matrix() = registration_result.transformation_;
    geometry_msgs::TransformStamped tf2_transform = tf2::eigenToTransform(at);
    tf2_pub.publish(tf2_transform);
  }

  std::pair<std::shared_ptr<open3d::geometry::PointCloud>,
            std::shared_ptr<open3d::pipelines::registration::Feature>>
  preprocess_pointcloud(open3d::geometry::PointCloud const &cloud,
                        float voxel_size) {

    // TODO: radius constants?
    using open3d::geometry::KDTreeSearchParamHybrid;
    using open3d::geometry::PointCloud;
    using open3d::pipelines::registration::ComputeFPFHFeature;
    using open3d::pipelines::registration::Feature;

    std::shared_ptr<PointCloud> cloud_down = cloud.VoxelDownSample(voxel_size);

    float radius_normal = voxel_size * 2;
    if (not cloud_down->HasNormals()) {
      cloud_down->EstimateNormals(KDTreeSearchParamHybrid(radius_normal, 30));
      cloud_down->OrientNormalsTowardsCameraLocation();
    }

    float radius_feature = voxel_size * 5;
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
    using open3d::pipelines::registration::TransformationEstimationPointToPoint;

    float voxel_size = 0.05; // 5cm
    float distance_threshold = voxel_size * 1.5;
    auto [cloud1_down, cloud1_fpfh] = preprocess_pointcloud(cloud1, voxel_size);
    auto [cloud2_down, cloud2_fpfh] = preprocess_pointcloud(cloud2, voxel_size);

    std::vector<std::reference_wrapper<const CorrespondenceChecker>> checkers;
    checkers.emplace_back(CorrespondenceCheckerBasedOnEdgeLength(0.9));
    checkers.emplace_back(
        CorrespondenceCheckerBasedOnDistance(distance_threshold));

    return RegistrationRANSACBasedOnFeatureMatching(
        *cloud1_down, *cloud2_down, *cloud1_fpfh, *cloud2_fpfh, true,
        distance_threshold, TransformationEstimationPointToPoint(false), 3,
        checkers, RANSACConvergenceCriteria(100000, 0.999));
  }
};
