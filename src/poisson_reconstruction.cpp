// #include <mesh_msgs/MeshGeometryStamped.h>
// #include <mesh_msgs_conversions/conversions.h>
// #include <pcl/surface/poisson.h>
// #include <ros/ros.h>
//
// void compute(const pcl::PCLPointCloud2::ConstPtr &input,
//              PolygonMesh &                        output,
//              int                                  depth,
//              int                                  solver_divide,
//              int                                  iso_divide,
//              float                                point_weight)
// {
//     PointCloud<PointNormal>::Ptr xyz_cloud(new
//     pcl::PointCloud<PointNormal>()); fromPCLPointCloud2(*input, *xyz_cloud);
//
//     print_info("Using parameters: depth %d, solverDivide %d, isoDivide %d\n",
//                depth,
//                solver_divide,
//                iso_divide);
//
//     Poisson<PointNormal> poisson;
//     poisson.setDepth(depth);
//     poisson.setSolverDivide(solver_divide);
//     poisson.setIsoDivide(iso_divide);
//     poisson.setPointWeight(point_weight);
//     poisson.setInputCloud(xyz_cloud);
//
//     TicToc tt;
//     tt.tic();
//     print_highlight("Computing ...");
//     poisson.reconstruct(output);
//
//     print_info("[Done, ");
//     print_value("%g", tt.toc());
//     print_info(" ms]\n");
// }
//
// class PoissonReconstructionNode
// {
//     std::string const cloud_topic = "camera/depth_registered/points";
//     // std::string const mesh_topic  = "poisson/reconstruction";
//
//     PoissonReconstructionNode(ros::NodeHandle &nh) : nh_(nh)
//     {
//         cloud_sub_ =
//             nh_.subscribe(cloud_topic,
//                           20,
//                           &PoissonReconstructionNode::pointCloud2Callback,
//                           this);
//         // mesh_pub_ =
//         nh_.advertise<mesh_msgs::MeshGeometryStamped>(mesh_topic,
//         // 1);
//     }
//
//   private:
//     ros::NodeHandle nh_;
//     ros::Subscriber cloud_sub_;
//     // ros::Publisher     mesh_pub_;
//     ros::ServiceServer service_;
//
//     void pointCloud2Callback(sensor_msgs::PointCloud2::ConstPtr const &cloud)
//     {
//         mesh_msgs::MeshGeometryStamped mesh_msg;
//         createMeshGeometryMessage(*cloud, mesh_msg);
//         // mesh_pub_.publish(mesh_msg);
//     }
//     bool createMeshGeometryMessage(sensor_msgs::PointCloud2 const &cloud,
//                                    mesh_msgs::MeshGeometryStamped &mesh_msg)
//     {
//         lvr2::MeshBufferPtr            mesh_buffer_ptr(new lvr2::MeshBuffer);
//         pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//         pcl::fromROSMsg(cloud, pcl_cloud);
//
//         // jeje xd
//         // diego me la chupa
//         if (!createMeshGeometry(pcl_cloud, jeje xd ))
//         {
//             ROS_ERROR_STREAM("Reconstruction Failed!");
//             return false;
//         }
//         if (!mesh_msgs_conversions::fromMeshBufferToMeshGeometryMessage(
//                 mesh_buffer_ptr, mesh_msg.mesh_geometry))
//         {
//
//             ROS_ERROR_STREAM("Could not convert point cloud from mesh_buffer
//             "
//                              "to Geometry Message!");
//             return false;
//         }
//         mesh_msg.header.frame_id = cloud.header.frame_id;
//         mesh_msg.header.stamp    = cloud.header.stamp;
//         mesh_msg.uuid            = std::to_string(cloud.header.seq);
//         return true;
//     }
// };
//
// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "poisson_reconstruction_node");
//     ros::NodeHandle           nh;
//     PoissonReconstructionNode rn(nh);
//     ros::spin();
//     return 0;
// }
