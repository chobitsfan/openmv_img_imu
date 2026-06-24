import time
import rclpy
import struct
import math
# import cv2
# import numpy as np
from openmv.camera import Camera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Int64

MG_TO_MSS = 9.80665 / 1000
MDPS_TO_RPS = math.pi / 180 / 1000

with open('acc_cali.csv', 'r') as f:
    acc_offset = tuple(float(x) for x in f.readline().split(','))
    acc_scale = tuple(float(x) for x in f.readline().split(','))

rclpy.init()
node = rclpy.create_node('openmv')
img_pub = node.create_publisher(Image, "mono_left", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
imu_pub = node.create_publisher(Imu, "imu", 200)
t_offset_pub = node.create_publisher(Int64, "pico_pi_t_offset", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))  # I use pico in the beginning

# The on-cam script above, stored as a string (or read from a file).
SCRIPT = open("frame_streamer_on_cam.py").read()

with Camera("/dev/ttyACM0", ack=False, crc=False) as cam:
    print(cam.system_info())
    # Stop running script (if any)
    cam.stop()
    time.sleep(0.5)
    cam.exec(SCRIPT)
    cam.streaming(False)
    print("ok")
#    img_i = 0
    frame_ch_id = None
    cnt = 0
    prv_imu_us = 0
    img_waiting = False

    while True:
        if text := cam.read_stdout():
            print("cam:", text)
        status = cam.read_status()
        if status.get("imu"):
            data = cam.channel_read("imu")
            # print(len(data))
            imu_samples = list(struct.iter_unpack("<Iffffff", data))
            # print(len(imu_samples))
            for imu_us, gx, gy, gz, ax, ay, az in imu_samples:
                if prv_imu_us > 0 and imu_us - prv_imu_us > 5000:
                    print("imu ts gap", imu_us - prv_imu_us)
                prv_imu_us = imu_us
                imu = Imu()
                imu.header.frame_id = "body"
                imu.header.stamp.sec = imu_us // 1_000_000
                imu.header.stamp.nanosec = (imu_us % 1_000_000) * 1000
                imu.linear_acceleration.x = (ax * MG_TO_MSS - acc_offset[0]) * acc_scale[0]
                imu.linear_acceleration.y = (ay * MG_TO_MSS - acc_offset[1]) * acc_scale[1]
                imu.linear_acceleration.z = (az * MG_TO_MSS - acc_offset[2]) * acc_scale[2]
                imu.angular_velocity.x = gx * MDPS_TO_RPS
                imu.angular_velocity.y = gy * MDPS_TO_RPS
                imu.angular_velocity.z = gz * MDPS_TO_RPS
                imu_pub.publish(imu)

        if img_waiting and prv_imu_us > img_us:
            img_waiting = False
            img_pub.publish(img)

        if status.get("frame"):
            if frame_ch_id is None:
                frame_ch_id = cam.get_channel(name="frame")
            h, w, img_us, cam_us = cam._channel_shape(frame_ch_id)
            cnt += 1
            if cnt > 50:
                cnt = 0
                now_ns = time.monotonic_ns()
                t_off = Int64()
                t_off.data = cam_us * 1000 - now_ns
                t_offset_pub.publish(t_off)

            data = cam.channel_read("frame")

#            cv_img = np.frombuffer(data, np.uint8).reshape(h, w)
#            cv2.imshow("OpenMV", cv_img)
#            k = cv2.waitKey(1)
#            if k == ord("q"):
#                break
#            elif k == ord("s"):
#                cv2.imwrite(f"openmv_{img_i}.png", cv_img)
#                img_i += 1

            img = Image()
            img.header.frame_id = "body"
            img.header.stamp.sec = img_us // 1000000
            img.header.stamp.nanosec = (img_us % 1000000) * 1000
            img.width = w
            img.height = h
            img.is_bigendian = 0
            img.encoding = "mono8"
            img.step = w
            img.data = data
            img_waiting = True

#    cv2.destroyAllWindows()
