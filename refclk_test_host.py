import time
from openmv.camera import Camera

# The on-cam script above, stored as a string (or read from a file).
SCRIPT = open("refclk_test_cam.py").read()

with Camera("/dev/ttyACM0", ack=False, crc=False) as cam:
    print(cam.system_info())
    # Stop running script (if any)
    cam.stop()
    time.sleep(0.5)
    cam.exec(SCRIPT)
    cam.streaming(False)
    print("ok")

    while True:
        if text := cam.read_stdout():
            print("cam:", text)
