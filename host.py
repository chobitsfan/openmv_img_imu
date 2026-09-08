import time
import rclpy
import struct
import math
import sys
import multiprocessing as mp
from multiprocessing import shared_memory
import my_img_pub
# import cv2
# import numpy as np
from openmv.camera import Camera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64

ACC_TO_MSS = 0.244 * 9.80665 / 1000  # +/-8 g -> 0.244 mg/LSB
GYRO_TO_RPS = 70 * math.pi / 180 / 1000  # 2000 dps -> 70 mdps/LSB

def main():
    if len(sys.argv) <= 1:
        print("need acc cali file")
        sys.exit()
    with open(sys.argv[1], 'r') as f:
        acc_offset = tuple(float(x) for x in f.readline().split(','))
        acc_scale = tuple(float(x) for x in f.readline().split(','))

    # Single-slot shared-memory handoff: the read loop must never block on the
    # publisher, so a frame arriving while the child is still copying is dropped.
    init_q = mp.Queue()
    frame_lock = mp.Lock()
    frame_ready = mp.Event()
    stop = mp.Event()
    frame_seq = mp.Value('L', 0, lock=False)   # guarded by frame_lock
    frame_ts = mp.Value('Q', 0, lock=False)
    proc = mp.Process(target=my_img_pub.run,
                      args=(init_q, frame_lock, frame_ready, stop, frame_seq, frame_ts))
    proc.start()
    shm = None
    n_collide = 0

    rclpy.init()
    node = rclpy.create_node('openmv')
    imu_pub = node.create_publisher(Imu, "imu", 400)
    t_offset_pub = node.create_publisher(Int64, "pico_pi_t_offset", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))  # I use pico in the beginning

    imu = Imu()
    imu.header.frame_id = "body"

    # The on-cam script above, stored as a string (or read from a file).
    SCRIPT = open("frame_streamer_on_cam.py").read()

    with Camera("/dev/ttyACM0", ack=False, crc=False) as cam, open("img_ts.csv", "w") as img_log, open("imu_ts.csv", "w") as imu_log:
    # with Camera("/dev/ttyACM0", ack=False, crc=False) as cam, open("openmv_ae3_acc_gyro.csv", "w") as log:
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
        imu_us_sum = 0
        # imu_sample_cnt = 0
        img_us = 0
        next_stdout_ns = 0  # read_stdout() costs a round trip; poll it at 1 Hz

        try:
            while True:
                if (now_ns := time.monotonic_ns()) >= next_stdout_ns:
                    next_stdout_ns = now_ns + 1_000_000_000
                    if text := cam.read_stdout():
                        print("cam:", text)
                status = cam.read_status()
                if status.get("imu"):
                    data = cam.channel_read("imu")
                    # print(len(data))
                    imu_samples = list(struct.iter_unpack("<hhhhhhI", data))
                    # print(len(imu_samples))
                    for gx, gy, gz, ax, ay, az, imu_us in imu_samples:
                        if prv_imu_us > 0:
                            diff_us = imu_us - prv_imu_us
                            imu_us_sum += diff_us
                            # imu_sample_cnt += 1
                            # if imu_sample_cnt > 600:
                            #    print("avg imu intl", imu_us_sum // imu_sample_cnt)
                            #    imu_us_sum = 0
                            #    imu_sample_cnt = 0
                            if diff_us > 4800 or diff_us < 4600:
                                print("imu ts gap", imu_us - prv_imu_us)
                        prv_imu_us = imu_us
                        imu.header.stamp.sec = imu_us // 1_000_000
                        imu.header.stamp.nanosec = (imu_us % 1_000_000) * 1000
                        imu.linear_acceleration.x = (ax * ACC_TO_MSS - acc_offset[0]) * acc_scale[0]
                        imu.linear_acceleration.y = (ay * ACC_TO_MSS - acc_offset[1]) * acc_scale[1]
                        imu.linear_acceleration.z = (az * ACC_TO_MSS - acc_offset[2]) * acc_scale[2]
                        imu.angular_velocity.x = gx * GYRO_TO_RPS
                        imu.angular_velocity.y = gy * GYRO_TO_RPS
                        imu.angular_velocity.z = gz * GYRO_TO_RPS
                        imu_pub.publish(imu)

                        # log.write(f"{imu_us},{imu.linear_acceleration.x:.10f},{imu.linear_acceleration.y:.10f},{imu.linear_acceleration.z:.10f},{imu_us},{imu.angular_velocity.x:.10f},{imu.angular_velocity.y:.10f},{imu.angular_velocity.z:.10f}\n")
                        imu_log.write(f"{imu_us}\n")

                if status.get("frame"):
                    if frame_ch_id is None:
                        frame_ch_id = cam.get_channel(name="frame")
                    h, w, img_us, cam_us = cam._channel_shape(frame_ch_id)
                    cnt += 1
                    if cnt > 50:
                        cnt = 0
                        t_off = Int64()
                        t_off.data = cam_us * 1000 - time.monotonic_ns()
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

                    if shm is None:
                        shm = shared_memory.SharedMemory(create=True, size=len(data))
                        init_q.put((shm.name, h, w, len(data)))
                    if frame_lock.acquire(block=False):
                        try:
                            shm.buf[:] = data
                            frame_ts.value = img_us
                            frame_seq.value += 1
                        finally:
                            frame_lock.release()
                        frame_ready.set()
                    else:
                        n_collide += 1  # child mid-copy; drop. Frames lost while the
                        # child is stalled are overwritten instead, and counted there.

                    img_log.write(f"{img_us}\n")

        except KeyboardInterrupt:
            cam.reset()

    #    cv2.destroyAllWindows()

    stop.set()
    frame_ready.set()
    proc.join()
    if shm is not None:
        shm.close()
        shm.unlink()

    print("frames dropped on slot collision:", n_collide)
    print("bye")

if __name__ == "__main__":
    mp.set_start_method("forkserver")
    main()
