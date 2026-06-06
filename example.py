import time
import rclpy
import struct
import math
from openmv.camera import Camera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, Imu

ACC_LSB_G   = 0.244 / 1000        # +/-8 g   -> 0.244 mg/LSB
GYR_LSB_DPS = 70.0 / 1000         # 2000 dps -> 70 mdps/LSB

rclpy.init()
node = rclpy.create_node('openmv')
img_pub = node.create_publisher(Image, "mono", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
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
    # prv_imu_ts = 0

    while True:
        if text := cam.read_stdout():
            print("cam:", text)
        status = cam.read_status()
        if status.get("imu"):
            data = cam.channel_read("imu")
            # count = len(data) // 4
            # print("rcv", len(data))
            tss = []
            imu_samples = list(struct.iter_unpack('hhhhhhi', data))
            # print('unpack', len(imu_samples))
            # i = 0
            # ts_diff = []
            for gx, gy, gz, ax, ay, az, ts in imu_samples:
                # print(gx*GYR_LSB_DPS, gy*GYR_LSB_DPS, gz*GYR_LSB_DPS, ax*ACC_LSB_G, ay*ACC_LSB_G, az*ACC_LSB_G, ts*0.025)
                # print(ts-prv_imu_ts)
                # if ts - prv_imu_ts > 200:
                #    print("imu ts gap", (ts - prv_imu_ts)*25//1000, "ms", prv_imu_ts, ts)
                # ts_diff.append(ts-prv_imu_ts)
                # prv_imu_ts = ts
                # i += 1
                tss.append(ts)
                imu = Imu()
                imu.header.frame_id = "body"
                imu.header.stamp.sec = ts*25 // 1_000_000
                imu.header.stamp.nanosec = (ts*25 % 1_000_000) * 1000
                imu.linear_acceleration.x = ax*ACC_LSB_G*9.80665
                imu.linear_acceleration.y = ay*ACC_LSB_G*9.80665
                imu.linear_acceleration.z = az*ACC_LSB_G*9.80665
                imu.angular_velocity.x = gx*GYR_LSB_DPS*math.pi/180
                imu.angular_velocity.y = gy*GYR_LSB_DPS*math.pi/180
                imu.angular_velocity.z = gz*GYR_LSB_DPS*math.pi/180
                imu_pub.publish(imu)
            print(tss)

        if status.get("frame"):
        # if False:
            h, w, img_ts = cam._channel_shape(cam.get_channel(name="frame"))

            data = cam.channel_read("frame")

            # pi_ts2 = time.monotonic_ns()
            # print((pi_ts2 - pi_ts)//1000000, 'ms')
            # pi_ts = pi_ts2
            # print(cam.read_stdout())

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
