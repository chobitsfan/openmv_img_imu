import csi
import protocol
import time

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.VGA)
csi0.framerate(20)

img = csi0.snapshot()
img_mv = memoryview(img.bytearray())
frame_ready = True

start_ts = time.ticks_us()
img_ts = time.ticks_us()

class FrameChannel:
    def size(self):
        return len(img_mv)

    def shape(self):
        return (img.height(), img.width(), time.ticks_diff(img_ts, start_ts))

    def poll(self):
        return frame_ready

    def readp(self, offset, size):
        global frame_ready
        end = offset + size
        chunk = img_mv[offset:end]
        if end >= len(img_mv):
            frame_ready = False
        return chunk

# protocol.init(ack=False)
protocol.register(name="frame", backend=FrameChannel())

while True:
    if not frame_ready:
        img = csi0.snapshot()
        img_ts = time.ticks_us()
        img_mv = memoryview(img.bytearray())
        frame_ready = True
