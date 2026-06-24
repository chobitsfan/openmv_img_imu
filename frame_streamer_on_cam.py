import csi
import protocol
import openamp
import refclk
import machine

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
        return len(imu_samples_xfer)

    def poll(self):
        global imu_samples_fill, imu_samples_xfer
        if imu_samples_fill:
            imu_samples_xfer = imu_samples_fill
            imu_samples_fill = bytearray()
            return True
        else:
            return False

    def read(self, offset, size):
        return imu_samples_xfer


def task_callback(src_addr, data):
    global imu_samples_fill
    imu_samples_fill += data


@openamp.async_remote(task_callback)
async def task1(ept):
    import imu
    import refclk
    import machine
    import struct

    drdy = False

    def imu_drdy_cb(pin):
        nonlocal drdy
        drdy = True
    imu.__write_reg(0x10, 0x5c)  # acc 208hz, 8g
    imu.__write_reg(0x11, 0x5c)  # gyro 208hz, 2000dps
    imu.__write_reg(0x0B, 0x80)  # pulsed DataReady
    imu.__write_reg(0x0D, 0x01)  # acc DataReady INT1
    machine.Pin('P15_4', mode=machine.Pin.IN).irq(handler=imu_drdy_cb, trigger=machine.Pin.IRQ_RISING, hard=True)
    while True:
        if drdy:
            drdy = False
            now_us = refclk.now_us()
            ax, ay, az = imu.acceleration_mg()
            gx, gy, gz = imu.angular_rate_mdps()
            ept.send(struct.pack("<Iffffff", now_us, gx, gy, gz, ax, ay, az))


def main():
    global img, img_us, img_mv, frame_ready
    refclk.enable()
    rproc = openamp.RemoteProc(0x80320000)
    rproc.start()

    last_trig_us = 0

    protocol.register(name="frame", backend=FrameChannel())
    protocol.register(name="imu", backend=ImuChannel())

    while True:
        machine.idle()
        if not frame_ready:
            now_us = refclk.now_us()
            if now_us - last_trig_us > 49000:
                csi0.snapshot(image=img)
                last_trig_us = now_us
                img_us = now_us + csi0.exposure_us() // 2
                img_mv = memoryview(img)
                frame_ready = True


if __name__ == '__main__':
    main()
