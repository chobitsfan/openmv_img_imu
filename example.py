import time
import rclpy
from openmv.camera import Camera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node('openmv')
my_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
img_pub = node.create_publisher(Image, "mono", my_qos)

# The on-cam script above, stored as a string (or read from a file).
SCRIPT = open("frame_streamer_on_cam.py").read()
ts = time.monotonic_ns()

with Camera("/dev/ttyACM0", ack=False, crc=False) as cam:
    print(cam.system_info())
    cam.stop()
    cam.exec(SCRIPT)

    while True:
        status = cam.read_status()
        if not cam.has_channel("frame") or not status.get("frame"):
            continue

        h, w, img_ts = cam._channel_shape(cam.get_channel(name="frame"))

        data = cam.channel_read("frame")

        # ts2 = time.monotonic_ns()
        # print((ts2 - ts)//1000000, 'ms', img_ts)
        # ts = ts2

        img = Image()
        img.header.frame_id = "body"
        img.header.stamp.sec = img_ts // 1000000
        img.header.stamp.nanosec = (img_ts % 1000000) * 1000
        img.width = w
        img.height = h
        img.is_bigendian = 0
        img.encoding = "mono8"
        img.step = w
        img.data = data
        img_pub.publish(img)
