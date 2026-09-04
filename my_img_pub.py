import math
import struct
import rclpy
from multiprocessing import shared_memory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Int64

ACC_TO_MSS = 0.244 * 9.80665 / 1000  # +/-8 g -> 0.244 mg/LSB
GYRO_TO_RPS = 70 * math.pi / 180 / 1000  # 2000 dps -> 70 mdps/LSB
IMU_REC = 16  # <hhhhhhI


def run(init_q, lock, ready, stop, seq, ts, imu_w, imu_r, toff_q):
    """All ROS publishing lives here so the camera read loop never touches DDS."""
    (name, h, w, nbytes, imu_name, imu_slots, acc_offset, acc_scale) = init_q.get()
    shm = shared_memory.SharedMemory(name=name)
    imu_shm = shared_memory.SharedMemory(name=imu_name)

    rclpy.init()
    node = rclpy.create_node('openmv2')
    img_pub = node.create_publisher(Image, "mono_left", QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT))
    imu_pub = node.create_publisher(Imu, "imu", 400)  # RELIABLE: may block, which is fine here
    t_offset_pub = node.create_publisher(Int64, "pico_pi_t_offset", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))

    img = Image()
    img.header.frame_id = "body"
    img.is_bigendian = 0
    img.width = w
    img.height = h
    img.step = w
    img.encoding = "mono8"
    imu = Imu()
    imu.header.frame_id = "body"
    t_off = Int64()

    buf = bytearray(nbytes)
    last = 0
    n_over = 0      # frames overwritten in the slot while we were behind
    n_imu_lost = 0  # imu records overwritten in the ring while we were behind
    try:
        while not stop.is_set():
            ready.wait(0.1)
            ready.clear()   # clear before reading, so work queued in between is not lost

            r, wv = imu_r.value, imu_w.value
            if wv - r > imu_slots:      # ring lapped; skip to the oldest surviving record
                n_imu_lost += wv - imu_slots - r
                r = wv - imu_slots
            while r < wv:
                gx, gy, gz, ax, ay, az, imu_us = struct.unpack_from(
                    "<hhhhhhI", imu_shm.buf, (r % imu_slots) * IMU_REC)
                imu.header.stamp.sec = imu_us // 1_000_000
                imu.header.stamp.nanosec = (imu_us % 1_000_000) * 1000
                imu.linear_acceleration.x = (ax * ACC_TO_MSS - acc_offset[0]) * acc_scale[0]
                imu.linear_acceleration.y = (ay * ACC_TO_MSS - acc_offset[1]) * acc_scale[1]
                imu.linear_acceleration.z = (az * ACC_TO_MSS - acc_offset[2]) * acc_scale[2]
                imu.angular_velocity.x = gx * GYRO_TO_RPS
                imu.angular_velocity.y = gy * GYRO_TO_RPS
                imu.angular_velocity.z = gz * GYRO_TO_RPS
                imu_pub.publish(imu)
                r += 1
            imu_r.value = r

            while True:
                try:
                    t_off.data = toff_q.get_nowait()
                except Exception:
                    break
                t_offset_pub.publish(t_off)

            with lock:
                if seq.value == last:
                    continue
                n_over += seq.value - last - 1
                last = seq.value
                img_us = ts.value
                buf[:] = shm.buf         # copy out, then release the slot
            img.header.stamp.sec = img_us // 1000000
            img.header.stamp.nanosec = (img_us % 1000000) * 1000
            img.data = buf
            img_pub.publish(img)
    except KeyboardInterrupt:
        pass
    finally:
        print("frames overwritten before publish:", n_over)
        print("imu records lost in ring:", n_imu_lost)
        shm.close()
        imu_shm.close()
