import csi
import protocol
import time
import imu
import struct
import machine
import micropython

micropython.alloc_emergency_exception_buf(100)

# --- LSM6DSM Register Addresses ---
CTRL1_XL = 0x10  # Accel control register
CTRL2_G = 0x11  # Gyro control register
CTRL3_C = 0x12  # Control register 3
CTRL10_C = 0x19  # Control register 10 (Timer enable)
INT1_CTRL = 0x0D  # INT1 pad control
FIFO_CTRL1 = 0x06  # FIFO watermark threshold FTH[7:0]
FIFO_CTRL2 = 0x07  # FIFO timer/pedo enable + FTH[10:8]
FIFO_CTRL3 = 0x08  # FIFO decimation for Accel/Gyro
FIFO_CTRL4 = 0x09  # FIFO decimation for Dataset 3/4
FIFO_CTRL5 = 0x0A  # FIFO ODR and Mode
FIFO_STATUS1 = 0x3A  # FIFO status 1 (number of unread words LSB)
FIFO_STATUS2 = 0x3B  # FIFO status 2 (number of unread words MSB + flags)
FIFO_DATA_OUT_L = 0x3E  # FIFO data output LSB
FIFO_DATA_OUT_H = 0x3F  # FIFO data output MSB
WAKE_UP_DUR = 0x5C  # Wake up duration / High-res timer
TIMESTAMP0_REG = 0x40
TIMESTAMP1_REG = 0x41
TIMESTAMP2_REG = 0x42


def read_fifo():
    status1 = imu.__read_reg(FIFO_STATUS1)
    status2 = imu.__read_reg(FIFO_STATUS2)

    if status2 & 0x40:
        print("FIFO overrun occurred!")

    # DIFF_FIFO is an 11-bit value representing unread 16-bit words
    words = (status1 | ((status2 & 0x07) << 8))

    imu_us = time.ticks_diff(imu_fifo_ts, start_ts)  # before read fifo, it may raise another irq

    # occasionally, DIFF_FIFO is not a full acc+gyro sample, floor to whole 6-word patterns
    nbytes = (words // 6) * 6 * 2
    data = bytearray(nbytes)
    imu.__read_reg(0x3E, data)

    data += imu_us.to_bytes(4, 'little')

    return data


def imu_fifo_cb(pin):
    global imu_fifo_ts, imu_fifo_reach_th
    imu_fifo_ts = time.ticks_us()
    imu_fifo_reach_th = True


class FrameChannel:
    def size(self):
        return len(img_mv)

    def shape(self):
        return (img.height(), img.width(), img_us)

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
        return len(imu_samples)

    def poll(self):
        return imu_ready

    def read(self, offset, size):
        global imu_ready
        imu_ready = False
        return imu_samples


# Reset flow per AN4987
imu.__write_reg(CTRL2_G,  0x00)        # 1. gyro -> Power-Down (ODR_G = 0000)
imu.__write_reg(CTRL1_XL, 0x50)        # 2. accel -> High-Performance (ODR=208Hz, FS=2g; any HP-mode ODR)
imu.__write_reg(CTRL3_C, 0x01)

# 50 µs is enough
time.sleep_ms(5)

# 1. Enable Block Data Update (BDU) and Auto-increment (IF_INC)
imu.__write_reg(CTRL3_C, 0x44)

# 2. Timestamp resolution (25 us / LSB)
# TIMER_HR (bit 4) = 1 in WAKE_UP_DUR register -> 0x10
# imu.__write_reg(WAKE_UP_DUR, 0x10)

# 3. Enable the Hardware Timestamp Timer AND the Embedded Digital Block
# TIMER_EN (bit 5) = 1, FUNC_EN (bit 2) = 1 -> 0x24
# imu.__write_reg(CTRL10_C, 0x24)

# 4. Set Accelerometer ODR to 208Hz, 8g
imu.__write_reg(CTRL1_XL, 0x5c)

# 5. Set Gyroscope ODR to 208Hz, 2000dps
imu.__write_reg(CTRL2_G, 0x5c)

# 6. Enable Timestamp routing to the FIFO at every Data-Ready (DRDY)
#    FTH[10:8] = 0 (upper 3 bits of the 90-byte watermark, see FIFO_CTRL1)
# imu.__write_reg(FIFO_CTRL2, 0x80)

# 6a. FIFO watermark. FTH unit is word (2 bytes), WaterM/INT1_FTH asserts when unread word >= FTH.
imu.__write_reg(FIFO_CTRL1, 30)

# 6b. Route the FIFO threshold (watermark) flag to the INT1 pin.
#     INT1_FTH = bit 3 -> 0x08. INT1 is push-pull, active-high (CTRL3_C defaults).
imu.__write_reg(INT1_CTRL, 0x08)

# 7. Set FIFO decimation: No decimation for Accel and Gyro (Datasets 1 & 2)
imu.__write_reg(FIFO_CTRL3, 0x09)

# 8. Set FIFO decimation: No decimation for Timestamp (Dataset 4)
# imu.__write_reg(FIFO_CTRL4, 0x08)

# 9. Set FIFO ODR to 208Hz and mode to Continuous
imu.__write_reg(FIFO_CTRL5, 0x2E)

csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.GRAYSCALE)
csi0.framesize(csi.VGA)
# csi0.framerate(20)
csi0.ioctl(csi.IOCTL_SET_TRIGGERED_MODE, True)

img = csi0.snapshot()
img_mv = memoryview(img)
frame_ready = False

imu_samples = bytearray()
imu_ready = False

protocol.register(name="frame", backend=FrameChannel())
protocol.register(name="imu", backend=ImuChannel())

machine.Pin('P15_4', mode=machine.Pin.IN).irq(handler=imu_fifo_cb, trigger=machine.Pin.IRQ_RISING, hard=True)

# reset timestamp count
imu.__write_reg(TIMESTAMP2_REG, 0xaa)
# bypass (FIFO_MODE=000) empties the FIFO and resets its pointers
imu.__write_reg(FIFO_CTRL5, 0x00)
time.sleep_ms(5)
imu.__write_reg(FIFO_CTRL5, 0x2E)

start_ts = time.ticks_us()
img_ts = time.ticks_us()
imu_fifo_ts = time.ticks_us()
imu_fifo_reach_th = False
imu_us = 0

while True:
    if imu_fifo_reach_th and (not imu_ready):
        imu_fifo_reach_th = False  # clear frist, read fifo may rise another irq
        imu_samples = read_fifo()
        imu_ready = True
    if not frame_ready:
        now_ts = time.ticks_us()
        if time.ticks_diff(now_ts, img_ts) > 49000:
            csi0.snapshot(image=img)
            img_us = time.ticks_diff(now_ts, start_ts) + csi0.exposure_us() // 2
            img_ts = now_ts
            img_mv = memoryview(img)
            frame_ready = True
