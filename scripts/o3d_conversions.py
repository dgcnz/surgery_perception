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
                          pose=Pose(Point(0.0, 0.0, 0.0),
                                    Quaternion(0, 0, 0, 1)),
                          scale=Vector3(1, 1, 1),
                          header=Header(frame_id='camera_link'),
                          color=ColorRGBA(0.0, 0.0, 1.0, 0.8)):
    triangles = np.asarray(o3d_mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    points = []
    for t in triangles:
        for i in range(3):
            points.append(vertices[t[i]])

    marker = Marker(ns='o3d',
                    points=points,
                    type=Marker.TRIANGLE_LIST,
                    action=Marker.ADD,
                    id=int(time.time()),
                    lifetime=rospy.Duration(),
                    pose=pose,
                    scale=scale,
                    header=header,
                    color=color,
                    frame_locked=False)
    return marker
