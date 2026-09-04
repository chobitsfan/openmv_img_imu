import time
import struct
import sys
import multiprocessing as mp
from multiprocessing import shared_memory
import my_img_pub
# import cv2
# import numpy as np
from openmv.camera import Camera

IMU_REC = 16          # <hhhhhhI
IMU_SLOTS = 8192      # ~37 s of ring at 216 Hz


def main():
    if len(sys.argv) <= 1:
        print("need acc cali file")
        sys.exit()
    with open(sys.argv[1], 'r') as f:
        acc_offset = tuple(float(x) for x in f.readline().split(','))
        acc_scale = tuple(float(x) for x in f.readline().split(','))

    # All ROS publishing runs in the child. This loop must never block on it:
    # frames go through a single shared-memory slot (newest wins), imu samples
    # through a shared-memory ring, and neither write can stall.
    init_q = mp.Queue()
    toff_q = mp.Queue()
    frame_lock = mp.Lock()
    ready = mp.Event()
    stop = mp.Event()
    frame_seq = mp.Value('L', 0, lock=False)   # guarded by frame_lock
    frame_ts = mp.Value('Q', 0, lock=False)
    imu_w = mp.Value('Q', 0, lock=False)       # written only here
    imu_r = mp.Value('Q', 0, lock=False)       # written only by the child
    proc = mp.Process(target=my_img_pub.run,
                      args=(init_q, frame_lock, ready, stop, frame_seq, frame_ts,
                            imu_w, imu_r, toff_q))
    proc.start()

    imu_shm = shared_memory.SharedMemory(create=True, size=IMU_SLOTS * IMU_REC)
    shm = None
    n_collide = 0

    # The on-cam script above, stored as a string (or read from a file).
    SCRIPT = open("frame_streamer_on_cam.py").read()

    with Camera("/dev/ttyACM0", ack=False, crc=False) as cam, open("img_ts.csv", "w") as img_log, open("imu_ts.csv", "w") as imu_log:
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
                    w = imu_w.value
                    for off in range(0, len(data) - IMU_REC + 1, IMU_REC):
                        slot = (w % IMU_SLOTS) * IMU_REC
                        imu_shm.buf[slot:slot + IMU_REC] = data[off:off + IMU_REC]
                        w += 1
                        imu_us = struct.unpack_from("<I", data, off + 12)[0]
                        if prv_imu_us > 0:
                            diff_us = imu_us - prv_imu_us
                            if diff_us > 4800 or diff_us < 4600:
                                print("imu ts gap", diff_us)
                        prv_imu_us = imu_us
                        imu_log.write(f"{imu_us}\n")
                    imu_w.value = w      # publish the records only once they are all written
                    ready.set()

                if status.get("frame"):
                    if frame_ch_id is None:
                        frame_ch_id = cam.get_channel(name="frame")
                    h, w_px, img_us, cam_us = cam._channel_shape(frame_ch_id)
                    cnt += 1
                    if cnt > 50:
                        cnt = 0
                        try:
                            toff_q.put_nowait(cam_us * 1000 - time.monotonic_ns())
                        except Exception:
                            pass

                    data = cam.channel_read("frame")

        #            cv_img = np.frombuffer(data, np.uint8).reshape(h, w_px)
        #            cv2.imshow("OpenMV", cv_img)
        #            k = cv2.waitKey(1)
        #            if k == ord("q"):
        #                break
        #            elif k == ord("s"):
        #                cv2.imwrite(f"openmv_{img_i}.png", cv_img)
        #                img_i += 1

                    if shm is None:
                        shm = shared_memory.SharedMemory(create=True, size=len(data))
                        init_q.put((shm.name, h, w_px, len(data), imu_shm.name,
                                    IMU_SLOTS, acc_offset, acc_scale))
                    if frame_lock.acquire(block=False):
                        try:
                            shm.buf[:] = data
                            frame_ts.value = img_us
                            frame_seq.value += 1
                        finally:
                            frame_lock.release()
                        ready.set()
                    else:
                        n_collide += 1  # child mid-copy; drop. Frames lost while the
                        # child is stalled are overwritten instead, and counted there.

                    img_log.write(f"{img_us}\n")

        except KeyboardInterrupt:
            cam.reset()

    #    cv2.destroyAllWindows()

    stop.set()
    ready.set()
    proc.join()
    if shm is not None:
        shm.close()
        shm.unlink()
    imu_shm.close()
    imu_shm.unlink()

    print("frames dropped on slot collision:", n_collide)
    print("bye")


if __name__ == "__main__":
    mp.set_start_method("forkserver")
    main()
