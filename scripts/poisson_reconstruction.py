from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import *  # convert float to uint32
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import time

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8,
    (rgb_uint32 & 0x000000ff))
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))


def pointcloud2_to_o3d(ros_cloud):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(
        pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb
        # Get xyz
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data
               ]  # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0]
                [IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
            rgb = [
                convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data
            ]
        else:
            rgb = [
                convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data
            ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(
            np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud


def o3dmesh_to_rvizmarker(o3d_mesh,
                          frame_id,
                          id=0,
                          pose=Pose(Point(0.0, 0.0, 0.0),
                                    Quaternion(0, 0, 0, 1)),
                          scale=Vector3(1, 1, 1),
                          color=ColorRGBA(0.0, 1.0, 0.0, 0.8)):
    header = Header(frame_id=frame_id)
    triangles = np.asarray(o3d_mesh.triangles)
    vertices = np.asarray(o3d_mesh.vertices)
    points = []
    for t in triangles:
        for vix in t:
            points.append(
                Point(vertices[vix][0], vertices[vix][1], vertices[vix][2]))

    marker = Marker(ns='o3d',
                    points=points,
                    type=Marker.TRIANGLE_LIST,
                    action=Marker.ADD,
                    id=id,
                    lifetime=rospy.Duration(0),
                    pose=pose,
                    scale=scale,
                    header=header,
                    color=color,
                    frame_locked=False)
    return marker


import open3d
import rospy
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


def compute_poisson_mesh(pcd):
    with open3d.utility.VerbosityContextManager(
            open3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = open3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=6, n_threads=8)
        return mesh


def compute_normals(pcd):
    pcd.normals = open3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals
    pcd.estimate_normals()
    pcd.orient_normals_towards_camera_location()
    return pcd


class PoissonReconstruction:
    def __init__(self, pointcloud2_topic: str, mesh_topic: str):
        rospy.Subscriber(pointcloud2_topic, PointCloud2, self.make_mesh)
        rospy.loginfo("Start")
        self.pub = rospy.Publisher(mesh_topic, Marker, queue_size=1)

    def make_mesh(self, pc2_cloud):
        rospy.loginfo("Reading Cloud")
        rospy.loginfo(pc2_cloud.header.frame_id)
        cloud = pointcloud2_to_o3d(pc2_cloud)
        cloud = cloud.voxel_down_sample(voxel_size=0.05)
        rospy.loginfo("Computing Normals")
        cloud_with_normals = compute_normals(cloud)
        rospy.loginfo("Computing Mesh")
        mesh = compute_poisson_mesh(cloud_with_normals)
        rospy.loginfo("Publishing Mesh")
        marker = o3dmesh_to_rvizmarker(mesh, pc2_cloud.header.frame_id)
        self.pub.publish(marker)
        # open3d.io.write_triangle_mesh(f"/home/dgcnz/{int(time.time())}.ply", mesh)
        rospy.loginfo("Done")


if __name__ == '__main__':
    pointcloud2_topic = '/camera/depth_registered/points'
    mesh_topic = '/open3d/reconstruction'
    node_name = 'poisson_repointcloud2_to_o3d'

    node = PoissonReconstruction(pointcloud2_topic, mesh_topic)
    rospy.init_node(node_name, anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
