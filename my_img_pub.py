import rclpy
from multiprocessing import shared_memory
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

def run(init_q, lock, ready, stop, seq, ts):
    shm_name, h, w, nbytes = init_q.get()   # parent sends the shape once, on frame 0
    shm = shared_memory.SharedMemory(name=shm_name)

    rclpy.init()
    node = rclpy.create_node('openmv2')
    img_pub = node.create_publisher(Image, "mono_left", QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT))
    img = Image()
    img.header.frame_id = "body"
    img.is_bigendian = 0
    img.width = w
    img.height = h
    img.step = w
    img.encoding = "mono8"
    buf = bytearray(nbytes)
    last = 0
    n_over = 0
    try:
        while not stop.is_set():
            ready.wait(0.1)
            ready.clear()                    # clear before reading seq, so a frame
            with lock:                       # published in between is not lost
                if seq.value == last:
                    continue
                n_over += seq.value - last - 1   # frames overwritten while stalled
                last = seq.value
                img_us = ts.value
                buf[:] = shm.buf             # copy out, then release the slot
            img.header.stamp.sec = img_us // 1000000
            img.header.stamp.nanosec = (img_us % 1000000) * 1000
            img.data = buf
            img_pub.publish(img)
    except KeyboardInterrupt:
        pass
    finally:
        print("frames overwritten before publish:", n_over)
        shm.close()
