import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

def run(conn):
    rclpy.init()
    node = rclpy.create_node('openmv2')
    img_pub = node.create_publisher(Image, "mono_left", QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT))
    img = Image()
    img.header.frame_id = "body"
    img.is_bigendian = 0
    img.width = 640
    img.height = 400
    img.step = 640
    img.encoding = "mono8"
    buf = bytearray(640*400)
    try:
        while True:
            img_us = conn.recv()
            conn.recv_bytes_into(buf)    # fills buf in place, no allocation
            img.header.stamp.sec = img_us // 1000000
            img.header.stamp.nanosec = (img_us % 1000000) * 1000
            img.data = buf
            img_pub.publish(img)
    except (EOFError, KeyboardInterrupt):
        pass                             # parent closed the pipe
    finally:
        conn.close()


