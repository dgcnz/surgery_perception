#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>      //hydro
#include <visualization_msgs/MarkerArray.h> //hydro

// PCL specific includes
#include "pcl_ros/transforms.h"
#include <pcl_conversions/pcl_conversions.h> //hydro

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char *argv[]) {
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "perception_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string world_frame = "camera_link";
  std::string camera_frame = "camera_link";
  std::string cloud_topic = "camera/depth_registered/points";

  /*
   * SETUP PUBLISHERS
   */
  ros::Publisher object_pub =
      nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("cluster_marker_array", 1);

  while (ros::ok()) {
    /*
     * LISTEN FOR POINTCLOUD
     */
    std::string topic = nh.resolveName(cloud_topic);
    ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "
                    << topic);
    sensor_msgs::PointCloud2::ConstPtr recent_cloud =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

    /*
     * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
     */
    tf::TransformListener listener;
    tf::StampedTransform stransform;
    try {
      listener.waitForTransform(world_frame, recent_cloud->header.frame_id,
                                ros::Time::now(), ros::Duration(6.0));
      listener.lookupTransform(world_frame, recent_cloud->header.frame_id,
                               ros::Time(0), stransform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    sensor_msgs::PointCloud2 transformed_cloud;
    //  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
    //               ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic,
    //               nh);
    pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud,
                                 transformed_cloud);

    /*
     * CONVERT POINTCLOUD ROS->PCL
     */
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(transformed_cloud, cloud);

    /* ========================================
     * Fill Code: VOXEL GRID
     * ========================================*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_ptr);
    voxel_filter.setLeafSize(float(0.002), float(0.002), float(0.002));
    voxel_filter.filter(*cloud_voxel_filtered);

    // ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << "
    // points"); ROS_INFO_STREAM("Downsampled cloud  with " <<
    // cloud_voxel_filtered->size() << " points");

    /* ========================================
     * Fill Code: PASSTHROUGH FILTER(S)
     * ========================================*/

    // filter in x

    pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_voxel_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-2, 2);
    pass_x.filter(xf_cloud);

    // filter in y

    pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xf_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-2, 2);
    pass_y.filter(yf_cloud);

    // filter in z
    pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(yf_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-2, 2);
    pass_z.filter(zf_cloud);

    /* ========================================
     * Fill Code: CROPBOX (OPTIONAL)
     * ========================================*/

    /* ========================================
     * Fill Code: PLANE SEGEMENTATION
     * ========================================*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(
        new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Create the segmentation object for the planar model and set all the
    // parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.010);
    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud(cropped_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      ROS_WARN_STREAM(
          "Could not estimate a planar model for the given dataset.");
      // break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cropped_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    ROS_INFO_STREAM("PointCloud representing the planar component: "
                    << cloud_plane->points.size() << " data points.");

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);

    /* ========================================
     * Fill Code: PUBLISH PLANE MARKER (OPTIONAL)
     * ========================================*/

    /* ========================================
     * Fill Code: EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL/RECOMMENDED)
     * ========================================*/

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    *cloud_filtered = *cloud_f;
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05); // 2cm
    ec.setMinClusterSize(150);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin();
           pit != it->indices.end(); pit++)
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::cout << "Cluster has " << cloud_cluster->points.size()
                << " points.\n";
      clusters.push_back(cloud_cluster);
      sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
      pc2_clusters.push_back(tempROSMsg);
    }

    /* ========================================
     * BOUNDING VOLUME MARKERS
     * ========================================*/

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(
        camera_frame, "/rviz_visual_markers"));

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (auto cluster : clusters) {
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D(*cluster, minPt, maxPt);
      visual_tools_->publishWireframeCuboid(
          pose, Eigen::Vector3d(minPt.x, minPt.y, minPt.z),
          Eigen::Vector3d(maxPt.x, maxPt.y, maxPt.z), rviz_visual_tools::GREEN);
    }
    visual_tools_->trigger();

    /* ========================================
     * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
     * ========================================*/

    /* ========================================
     * Fill Code: PUBLISH OTHER MARKERS (OPTIONAL)
     * ========================================*/

    /* ========================================
     * BROADCAST TRANSFORM (OPTIONAL)
     * ========================================*/

    /* ========================================
     * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
     * ========================================*/

    /* ========================================
     * CONVERT POINTCLOUD PCL->ROS
     * PUBLISH CLOUD
     * Fill Code: UPDATE AS NECESSARY
     * ========================================*/
    sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
    // pcl::toROSMsg(*cloud_ptr, *pc2_cloud);
    // cloud_voxel_filtered
    // pcl::toROSMsg(*cloud_voxel_filtered, *pc2_cloud);
    // filters
    // pcl::toROSMsg(zf_cloud, *pc2_cloud);
    // plane segmentation
    pcl::toROSMsg(*cloud_f, *pc2_cloud);
    // pcl::toROSMsg(*(clusters.at(0)), *pc2_cloud);
    pc2_cloud->header.frame_id = world_frame;
    pc2_cloud->header.stamp = ros::Time::now();
    object_pub.publish(pc2_cloud);
  }
  return 0;
}
