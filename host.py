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

ACC_LSB_G   = 0.244 / 1000        # +/-8 g   -> 0.244 mg/LSB
GYR_LSB_DPS = 70.0 / 1000         # 2000 dps -> 70 mdps/LSB
ACC_TO_MSS = ACC_LSB_G * 9.80665
GYRO_TO_RPS = GYR_LSB_DPS * math.pi / 180
# IMU_INTERVAL_US = round(1_000_000 / 208)
IMU_INTERVAL_US = 4636

with open('acc_cali.csv', 'r') as f:
    acc_offset = tuple(float(x) for x in f.readline().split(','))
    acc_scale = tuple(float(x) for x in f.readline().split(','))

rclpy.init()
node = rclpy.create_node('openmv')
img_pub = node.create_publisher(Image, "mono_left", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
imu_pub = node.create_publisher(Imu, "imu", 200)

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
    # pi_ts = time.monotonic_ns()
#    img_i = 0
    img_waiting = False
    imu_us_5th = 0
    prv_imu_us = 0

    while True:
        if text := cam.read_stdout():
            print("cam:", text)
        status = cam.read_status()
        if status.get("imu"):
            data = cam.channel_read("imu")
            # print(len(data))
            imu_us_5th = struct.unpack_from('<I', data, len(data) - 4)[0]
            imu_samples = list(struct.iter_unpack('<hhhhhh', data[:-4]))
            if len(imu_samples) < 5:
                imu_us = imu_us_5th
            else:
                imu_us = imu_us_5th - IMU_INTERVAL_US * 4
            if imu_us - prv_imu_us > 5000 or imu_us - prv_imu_us < 4000:
                print(len(imu_samples), imu_us - prv_imu_us)
            for gx, gy, gz, ax, ay, az in imu_samples:
                prv_imu_us = imu_us
                imu = Imu()
                imu.header.frame_id = "body"
                imu.header.stamp.sec = imu_us // 1_000_000
                imu.header.stamp.nanosec = (imu_us % 1_000_000) * 1000
                imu.linear_acceleration.x = (ax * ACC_TO_MSS - acc_offset[0]) * acc_scale[0]
                imu.linear_acceleration.y = (ay * ACC_TO_MSS - acc_offset[1]) * acc_scale[1]
                imu.linear_acceleration.z = (az * ACC_TO_MSS - acc_offset[2]) * acc_scale[2]
                imu.angular_velocity.x = gx * GYRO_TO_RPS
                imu.angular_velocity.y = gy * GYRO_TO_RPS
                imu.angular_velocity.z = gz * GYRO_TO_RPS
                imu_pub.publish(imu)
                imu_us += IMU_INTERVAL_US

        if img_waiting and imu_us_5th > img_us:
            img_waiting = False
            img_pub.publish(img)

        if status.get("frame"):
            h, w, img_us = cam._channel_shape(cam.get_channel(name="frame"))

            data = cam.channel_read("frame")

            # pi_ts2 = time.monotonic_ns()
            # print((pi_ts2 - pi_ts)//1000000, 'ms')
            # pi_ts = pi_ts2

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
