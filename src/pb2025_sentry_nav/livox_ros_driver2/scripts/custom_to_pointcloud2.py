#!/usr/bin/env python3
"""
CustomMsg to PointCloud2 converter for Livox LiDAR visualization in RViz.
Subscribes to /livox/lidar (CustomMsg) and publishes /livox/lidar_pointcloud2 (PointCloud2).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import struct


class CustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('custom_to_pointcloud2')

        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/livox/lidar_pointcloud2',
            10
        )

        self.get_logger().info('CustomMsg to PointCloud2 converter started')
        self.get_logger().info('Subscribing to: /livox/lidar (CustomMsg)')
        self.get_logger().info('Publishing to: /livox/lidar_pointcloud2 (PointCloud2)')

    def callback(self, msg):
        """Convert CustomMsg to PointCloud2."""
        pc2 = PointCloud2()

        # Copy header
        pc2.header = msg.header

        # Set dimensions
        pc2.height = 1
        pc2.width = len(msg.points)

        # Define fields: x, y, z, intensity
        pc2.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc2.is_bigendian = False
        pc2.point_step = 16  # 4 fields * 4 bytes
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True

        # Pack point data
        data = []
        for point in msg.points:
            data.append(struct.pack('ffff', point.x, point.y, point.z, float(point.reflectivity)))

        pc2.data = b''.join(data)

        # Publish
        self.publisher.publish(pc2)


def main(args=None):
    rclpy.init(args=args)
    node = CustomToPointCloud2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
