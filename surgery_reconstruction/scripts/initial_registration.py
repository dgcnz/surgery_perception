import open3d as o3d
import numpy as np
import message_filters
import rospy
import copy

from open3d_ros_helper import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def manual_registration(source, target):
    source = source.remove_non_finite_points()
    target = target.remove_non_finite_points()
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    return reg_p2p.transformation


class InitialRegistrationNode:
    cloud_01_topic: str = '/camera_01/depth_registered/points'
    cloud_02_topic: str = '/camera_02/depth_registered/points'
    camera_01_sub: message_filters.Subscriber = None
    camera_02_sub: message_filters.Subscriber = None
    time_synchronizer: message_filters.ApproximateTimeSynchronizer = None
    broadcaster: tf2_ros.StaticTransformBroadcaster = None

    def __init__(self):
        self.camera_01_sub = message_filters.Subscriber(
            self.cloud_01_topic, PointCloud2)
        self.camera_02_sub = message_filters.Subscriber(
            self.cloud_02_topic, PointCloud2)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.camera_01_sub, self.camera_02_sub], 1, 0.1)
        self.ts.registerCallback(self.callback)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.init_node('initial_registration', anonymous=True)

    def callback(self, camera_01_cloud: PointCloud2,
                 camera_02_cloud: PointCloud2):
        transformation: np.ndmatrix = manual_registration(
            orh.rospc_to_o3dpc(camera_01_cloud),
            orh.rospc_to_o3dpc(camera_02_cloud))

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = camera_01_cloud.header.frame_id
        msg.child_frame_id = camera_02_cloud.header.frame_id
        msg.transform = orh.se3_to_transform(transformation)
        broadcaster.sendTransform(msg)
        self.camera_01_cloud.shutdown()
        self.camera_02_cloud.shutdown()


if __name__ == '__main__':
    node = InitialRegistrationNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
