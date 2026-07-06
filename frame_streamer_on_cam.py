import csi
import protocol
import openamp
import refclk
import machine
import struct
# import time

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
buf_a = bytearray(256)
buf_b = bytearray(256)
mv_fill = memoryview(buf_a)
mv_xfer = memoryview(buf_b)
fill_sz = 0
xfer_sz = 0
cnt = 0
imu_intl_us = 0
trig_us = 0


class FrameChannel:
    def size(self):
        return len(img_mv)

    def shape(self):
        return (img.height(), img.width(), img_us, refclk.now_us())

    def poll(self):
        return frame_ready

    def readp(self, offset, size):
        global frame_ready
        frame_ready = False
        return img_mv


class ImuChannel:
    def size(self):
        return xfer_sz

    def poll(self):
        return imu_ready

    def readp(self, offset, size):
        return mv_xfer[:xfer_sz]

    def read_done(self):
        global imu_ready
        imu_ready = False


def task_callback(src_addr, data):
    global mv_fill, mv_xfer, imu_ready, fill_sz, xfer_sz, cnt, imu_intl_us, trig_us
    if fill_sz <= len(mv_fill) - 16:
        mv_fill[fill_sz:fill_sz+16] = data
        fill_sz += 16
    if fill_sz >= 80 and not imu_ready:
        if imu_intl_us == 0:
            ts1 = struct.unpack_from("<I", mv_fill, 12)[0]
            ts5 = struct.unpack_from("<I", mv_fill, 76)[0]
            imu_intl_us = (ts5 - ts1) // 4
            print("imu_intl_us", imu_intl_us)
        mv_fill, mv_xfer = mv_xfer, mv_fill
        xfer_sz = fill_sz
        fill_sz = 0
        imu_ready = True
    cnt += 1
    if cnt == 10:
        cnt = 0
        if imu_intl_us:
            kf_us = struct.unpack_from("<I", data, 12)[0]
            trig_us = kf_us + 10 * imu_intl_us - csi0.exposure_us() // 2


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
    # imu.__write_reg(0x10, 0x5c)  # acc 208hz, 8g
    # imu.__write_reg(0x11, 0x5c)  # gyro 208hz, 2000dps
    imu.__write_reg(0x0B, 0x80)  # pulsed DataReady
    imu.__write_reg(0x0D, 0x01)  # acc DataReady INT1
    while True:
        await asyncio.sleep_ms(1)
        if drdy and (imu.__read_reg(0x1E) & 0x3) == 0x3:
            drdy = False
            imu.__read_reg(0x22, buf, 12)
            ept.send(buf)


def main():
    global img, img_us, img_mv, frame_ready, trig_us
    refclk.enable()
    rproc = openamp.RemoteProc(0x80320000)
    rproc.start()

    protocol.register(name="frame", backend=FrameChannel())
    protocol.register(name="imu", backend=ImuChannel())

    while True:
        machine.idle()
        now_us = refclk.now_us()
        if now_us >= trig_us:
            try:
                img = csi0.snapshot()
            except RuntimeError:
                print("snapshot failed")
                continue
            trig_us = 0
            img_mv = memoryview(img)
            img_us = now_us + csi0.exposure_us() // 2
            frame_ready = True


if __name__ == '__main__':
    main()
