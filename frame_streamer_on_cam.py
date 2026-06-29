import csi
import protocol
import openamp
import refclk
import machine
# import time

imu_samples_fill = bytearray()
imu_samples_xfer = bytearray()

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.VGA)
# csi0.framerate(20)
csi0.ioctl(csi.IOCTL_SET_TRIGGERED_MODE, True)
img = csi0.snapshot()
img_mv = memoryview(img)
frame_ready = False
img_us = 0
imu_ready = False
buf_fill = bytearray(256)
buf_xfer = bytearray(256)
fill_sz = 0
xfer_sz = 0


class FrameChannel:
    def size(self):
        return len(img_mv)

    def shape(self):
        return (img.height(), img.width(), img_us, refclk.now_us())

    def poll(self):
        return frame_ready

    def readp(self, offset, size):
        global frame_ready
        end = offset + size
        chunk = img_mv[offset:end]
        if end >= len(img_mv):
            frame_ready = False
        return chunk


class ImuChannel:
    def size(self):
        return xfer_sz

    def poll(self):
        return imu_ready

    def read(self, offset, size):
        global imu_ready
        out = buf_xfer[:xfer_sz]
        imu_ready = False
        return out


def task_callback(src_addr, data):
    global buf_fill, buf_xfer, imu_ready
    buf_fill[fill_sz:fill_sz+16] = data
    fill_sz += 16
    if fill_sz >= 80 and not imu_ready:
        buf_fill, buf_xfer = buf_xfer, buf_fill
        xfer_sz = fill_sz
        fill_sz = 0
        imu_ready = True


@openamp.async_remote(task_callback)
async def task1(ept):
    import imu
    import refclk
    import machine
    import asyncio

    buf = bytearray(16)
    imu_us_mv = memoryview(buf)[-4:]
    drdy = False

    def imu_drdy_cb(pin):
        nonlocal drdy
        refclk.now_us(imu_us_mv)
        drdy = True

    machine.Pin('P15_4', mode=machine.Pin.IN).irq(handler=imu_drdy_cb, trigger=machine.Pin.IRQ_RISING, hard=True)
    imu.__write_reg(0x10, 0x5c)  # acc 208hz, 8g
    imu.__write_reg(0x11, 0x5c)  # gyro 208hz, 2000dps
    # imu.__write_reg(0x0B, 0x80)  # pulsed DataReady
    imu.__write_reg(0x0D, 0x01)  # acc DataReady INT1
    while True:
        if drdy and (imu.__read_reg(0x1E) & 0x3) == 0x3:
            drdy = False
            imu.__read_reg(0x22, buf, 12)
            ept.send(buf)
            await asyncio.sleep_ms(1)


def main():
    global img, img_us, img_mv, frame_ready
    refclk.enable()
    rproc = openamp.RemoteProc(0x80320000)
    rproc.start()

    last_trig_us = 0

    protocol.register(name="frame", backend=FrameChannel())
    protocol.register(name="imu", backend=ImuChannel())

    while True:
        if not frame_ready:
            now_us = refclk.now_us()
            if now_us - last_trig_us > 49000:
                img = csi0.snapshot()
                img_mv = memoryview(img)
                last_trig_us = now_us
                img_us = now_us + csi0.exposure_us() // 2
                frame_ready = True
                machine.idle()


if __name__ == '__main__':
    main()
