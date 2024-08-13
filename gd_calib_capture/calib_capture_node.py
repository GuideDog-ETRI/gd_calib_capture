import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import cv2
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import os
import struct
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.executors import ExternalShutdownException
class CalibCaptureNode(Node):

    def __init__(self):
        super().__init__('data_saver_node')

        self.bridge = CvBridge()

        self.declare_parameter('camera_topic', '/camera_topic')
        self.declare_parameter('lidar_topic', '/lidar_topic')
        self.declare_parameter('image_file_prefix', 'image')
        self.declare_parameter('pcd_file_prefix', 'pointcloud')

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.image_file_prefix = self.get_parameter('image_file_prefix').get_parameter_value().string_value
        self.pcd_file_prefix = self.get_parameter('pcd_file_prefix').get_parameter_value().string_value

        self.image_subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            lidar_topic,
            self.lidar_callback,
            10)
        self.time = self.get_clock().now().to_msg().sec
        self.camera_received = False
        self.lidar_received = False

        self.point_cloud = []

    def image_callback(self, msg):
        self.get_logger().info('Received camera data')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = '{}_{}.png'.format(self.image_file_prefix, self.time)
        cv2.imwrite(filename, cv_image)
        self.camera_received = True
        self.check_all_received()


    # def lidar_callback(self, msg):
    #     self.get_logger().info('Received LiDAR data')
    #     point_cloud = self.convert_pointcloud2_to_open3d(msg)
    #     filename = '{}_{}.pcd'.format(self.pcd_file_prefix, self.time)
    #     o3d.io.write_point_cloud(filename, point_cloud)
    #     self.get_logger().info('Saved point cloud to {}'.format(filename))
    #     self.lidar_received = True
    #     self.check_all_received()

    def convert_image_msg_to_cv2(self, img_msg):
        height = img_msg.height
        width = img_msg.width
        channels = 3  # Assuming RGB8 encoding
        bytes_per_channel = 1
        img_data = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, width, channels)
        return img_data

    def lidar_callback(self, msg):
        self.get_logger().info('Received LiDAR data')
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            self.point_cloud.append([point[0], point[1], point[2]])
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.array(self.point_cloud))
        filename = '{}_{}.pcd'.format(self.pcd_file_prefix, self.time)
        o3d.io.write_point_cloud(filename, pc)
        self.lidar_received = True
        self.check_all_received()

    # def convert_pointcloud2_to_open3d(self, cloud_msg):
    #     points = []
    #     for point in self.read_points(cloud_msg, skip_nans=True):
    #         points.append([point[0], point[1], point[2]])
    #     cloud = o3d.geometry.PointCloud()
    #     cloud.points = o3d.utility.Vector3dVector(np.array(points))
    #     return cloud

    # def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
    #     fmt = self._get_struct_fmt(cloud)
    #     unpack_from = struct.Struct(fmt).unpack_from

    #     for p in self._read_points(cloud, field_names, skip_nans, uvs, unpack_from):
    #         yield p

    def check_all_received(self):
        if self.camera_received and self.lidar_received:
            self.get_logger().info('All data received and saved. Shutting down node.')
            raise SystemExit
            # rclpy.shutdown()

    # def _get_struct_fmt(self, cloud):
    #     fmt = '>' if cloud.is_bigendian else '<'
    #     offset = 0
    #     for field in cloud.fields:
    #         while offset < field.offset:
    #             fmt += 'x'
    #             offset += 1
    #         if field.datatype == 7:  # FLOAT32
    #             fmt += 'f'
    #         offset += 4
    #     return fmt
    
    
    # def _read_points(self, cloud, field_names, skip_nans, uvs, unpack_from):
    #     point_step = cloud.point_step
    #     row_step = cloud.row_step
    #     data = cloud.data

    #     for row in range(cloud.height):
    #         for col in range(cloud.width):
    #             point_offset = (row * row_step) + (col * point_step)
    #             point = unpack_from(data[point_offset:point_offset + point_step])
    #             yield point


def main(args=None):
    rclpy.init(args=args)
    node = CalibCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    rclpy.init(args=args)
    node = CalibCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()