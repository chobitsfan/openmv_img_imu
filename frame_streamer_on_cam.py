import csi
import protocol
import openamp
import refclk
import machine
import struct
from micropython import const
from machine import Pin
# import time


class FrameChannel:
    def size(self):
        return FRAME_SZ

    def shape(self):
        return (FRAME_H, FRAME_W, img_us, refclk.now_us())

    def poll(self):
        return frame_ready

    def readp(self, offset, size):
        global frame_ready
        frame_ready = False
        return img_mv


class ImuChannel:
    def size(self):
        return IMU_CHUNK_SZ

    def poll(self):
        # readp() always hands out IMU_CHUNK_SZ bytes, so only advertise a full chunk
        return (imu_wh - imu_rh) % IMU_BUF_SZ >= IMU_CHUNK_SZ

    def readp(self, offset, size):
        global imu_rh
        rh_t = imu_rh
        imu_rh = (imu_rh + IMU_CHUNK_SZ)  % IMU_BUF_SZ
        return imu_buf_mv[rh_t:rh_t + IMU_CHUNK_SZ]


def task_callback(src_addr, data):
    global imu_wh, cnt, imu_intl_us, trig_us, ts0_us, n_est, imu_ovf

    imu_buf_mv[imu_wh:imu_wh+16] = data
    imu_wh = (imu_wh + 16) % IMU_BUF_SZ
    if imu_wh == imu_rh:
        # write head lapped the read head: the ring now looks empty to poll()
        if not imu_ovf:
            imu_ovf = True
            print("imu ring overflow")
    else:
        imu_ovf = False

    # Average the IMU interval over a wide window (once) so the 10-interval
    # prediction lands on the true keyframe instead of ~65us early.
    if imu_intl_us == 0:
        ts = struct.unpack_from("<I", data, 12)[0]
        if ts0_us == 0:
            ts0_us = ts
        else:
            n_est += 1
            if n_est == EST_WIN:
                imu_intl_us = (ts - ts0_us) // EST_WIN
                print("imu_intl_us", imu_intl_us)
    cnt += 1
    if cnt == FRAME_INTL:
        cnt = 0
        if imu_intl_us:
            kf_us = struct.unpack_from("<I", data, 12)[0]
            # Kwabena: AEC will update its internal settings based on what is seen in that image unless you force manual control
            trig_us = kf_us + FRAME_INTL * imu_intl_us - csi0.exposure_us() // 2


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


def board_reboot(pin):
    machine.reset()


def main():
    global img, img_us, img_mv, frame_ready
    refclk.enable()
    rproc = openamp.RemoteProc(0x80320000)
    rproc.start()

    frame_ch = protocol.register(name="frame", backend=FrameChannel())
    imu_ch = protocol.register(name="imu", backend=ImuChannel())

    skip_cnt = 0
    last_trig = 0  # only task_callback writes trig_us; avoids clobbering a new trigger

    while True:
        now_us = refclk.now_us()
        t = trig_us
        if t and t != last_trig and now_us >= t:
            last_trig = t
            if frame_ready:
                skip_cnt += 1
                print("snapshot skipped", skip_cnt)
                continue
            exposure_us = csi0.exposure_us()
            try:
                img = csi0.snapshot()
            except RuntimeError:
                print("snapshot failed")
                continue
            img_mv = memoryview(img)
            img_us = now_us + exposure_us // 2
            frame_ready = True


csi0 = csi.CSI()
csi0.reset()
csi0.ioctl(csi.IOCTL_SET_TRIGGERED_MODE, True)
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.VGA)
# Kwabena: There’s no frame rate in triggered mode; the frame rate is how fast you can trigger. The framerate() option itself does, though, set an upper limit
csi0.framerate(100)
Pin("SW", Pin.IN).irq(board_reboot, Pin.IRQ_RISING, hard = True)
img = csi0.snapshot()
img_mv = memoryview(img)
frame_ready = False
img_us = 0
IMU_CHUNK_SZ = const(64)
IMU_BUF_SZ = const(512)
imu_buf = bytearray(IMU_BUF_SZ)
imu_buf_mv = memoryview(imu_buf)
imu_rh = 0
imu_wh = 0
imu_ovf = False
cnt = 0
imu_intl_us = 0
trig_us = 0
ts0_us = 0
n_est = 0
EST_WIN = const(256)  # intervals to average for imu_intl_us (~1.2s @ 215Hz)
FRAME_INTL = const(10)
FRAME_SZ = img.size()
FRAME_H = img.height()
FRAME_W = img.width()

if __name__ == '__main__':
    main()
