import open3d
import rospy
import numpy as np
import time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from ctypes import *  # convert float to uint32

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


def ros_to_open3d(ros_cloud):
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
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud


def compute_poisson_mesh(pcd):
    with open3d.utility.VerbosityContextManager(
            open3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = open3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9)
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
        # self.pub = rospy.Publisher(mesh_topic, , queue_size=10)

    def make_mesh(self, pc2_cloud):
        rospy.loginfo("Reading Cloud")
        cloud = ros_to_open3d(pc2_cloud)
        rospy.loginfo("Computing Normals")
        cloud_with_normals = compute_normals(cloud)
        rospy.loginfo("Computing Mesh")
        mesh = compute_poisson_mesh(cloud_with_normals)
        rospy.loginfo("Saving Mesh")
        open3d.io.write_triangle_mesh(f"/home/dgcnz/{int(time.time())}.ply", mesh)
        rospy.loginfo("Saving Mesh")

if __name__ == '__main__':
    pointcloud2_topic = '/camera/depth_registered/points'
    mesh_topic = '/open3d/reconstruction'
    node_name = 'poisson_reconstruction'

    node = PoissonReconstruction(pointcloud2_topic, mesh_topic)
    rospy.init_node(node_name, anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
