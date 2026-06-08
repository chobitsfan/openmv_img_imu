import csi
import protocol
import time
import imu
import struct

# --- LSM6DSM Register Addresses ---
CTRL1_XL = 0x10  # Accel control register
CTRL2_G = 0x11  # Gyro control register
CTRL3_C = 0x12  # Control register 3
CTRL10_C = 0x19  # Control register 10 (Timer enable)
FIFO_CTRL2 = 0x07  # FIFO timer/pedo enable
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


def read_current_timestamp():
    """
    Reads the current 24-bit timestamp directly from the hardware registers.
    """
    # Read the 3 bytes sequentially
    ts0 = imu.__read_reg(TIMESTAMP0_REG)
    ts1 = imu.__read_reg(TIMESTAMP1_REG)
    ts2 = imu.__read_reg(TIMESTAMP2_REG)

    # Combine the 3 bytes into a single 24-bit integer
    # ts0 is bits [7:0]
    # ts1 is bits [15:8]
    # ts2 is bits [23:16]
    timestamp_24bit = ts0 | (ts1 << 8) | (ts2 << 16)

    return timestamp_24bit


def read_fifo_word():
    """
    Reads a single 16-bit word from the FIFO data registers
    and converts it to a signed integer.
    """
    low = imu.__read_reg(FIFO_DATA_OUT_L)
    high = imu.__read_reg(FIFO_DATA_OUT_H)

    # Combine into a 16-bit signed integer (Two's Complement)
    val = (high << 8) | low
    if val >= 32768:
        val -= 65536
    return val


def read_fifo_word_unsigned():
    """Reads a 16-bit word from the FIFO as an unsigned integer."""
    low = imu.__read_reg(FIFO_DATA_OUT_L)
    high = imu.__read_reg(FIFO_DATA_OUT_H)
    return (high << 8) | low


def read_fifo_continuous_with_timestamp():
    status1 = imu.__read_reg(FIFO_STATUS1)
    status2 = imu.__read_reg(FIFO_STATUS2)

    if status2 & 0x40:
        print("FIFO overrun occurred!")

    # DIFF_FIFO is an 11-bit value representing unread 16-bit words
    unread_words = status1 | ((status2 & 0x07) << 8)

    # With Gyro, Accel, and Timestamp enabled, each complete sample set is 9 words long:
    # - 3 words (6 bytes) for Gyro
    # - 3 words (6 bytes) for Accel
    # - 3 words (6 bytes) for the Timestamp dataset
    sample_sets = unread_words // 9

    # prv_ts = 0
    data = bytearray()
    for _ in range(sample_sets):
        # 1. Read Gyroscope
        gd = imu.__read_reg_burst(0x3E, 6)
        gx, gy, gz = struct.unpack('<hhh', gd)

        # 2. Read Accelerometer
        ad = imu.__read_reg_burst(0x3E, 6)
        ax, ay, az = struct.unpack('<hhh', ad)

        # 3. Read Timestamp Dataset (3 words)
        td = imu.__read_reg_burst(0x3E, 6)
        ts_word0, ts_word1, _ = struct.unpack('<HHH', td)

        timestamp_24bit = ((ts_word0 << 8) | (ts_word1 >> 8)) & 0xFFFFFF
        # if prv_ts > 0 and timestamp_24bit - prv_ts > 200:
        #    print("ts gap", timestamp_24bit - prv_ts)
        # prv_ts = timestamp_24bit

        data += struct.pack('hhhhhhi', gx, gy, gz, ax, ay, az, timestamp_24bit)

    return data


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


class ImuChannel:
    def size(self):
        return len(imu_samples)

    def poll(self):
        return imu_ready

    def read(self, offset, size):
        global imu_ready
        imu_ready = False
        # if size < len(imu_samples):
        #    print("buf too small")
        return imu_samples


# 0. PERFORM SOFTWARE RESET
# SW_RESET (bit 0) = 1 -> 0x01
imu.__write_reg(CTRL3_C, 0x01)

# Wait for the sensor to complete its internal boot procedure
# (The datasheet states boot time is typically around 15ms)
time.sleep_ms(30)

# 1. Enable Block Data Update (BDU) and Auto-increment (IF_INC)
imu.__write_reg(CTRL3_C, 0x44)

# 2. Timestamp resolution (25 us / LSB)
# TIMER_HR (bit 5) = 1 in WAKE_UP_DUR register -> 0x10
imu.__write_reg(WAKE_UP_DUR, 0x10)

# 3. Enable the Hardware Timestamp Timer AND the Embedded Digital Block
# TIMER_EN (bit 5) = 1, FUNC_EN (bit 2) = 1 -> 0x24
imu.__write_reg(CTRL10_C, 0x24)

# 4. Set Accelerometer ODR to 208Hz, 8g
imu.__write_reg(CTRL1_XL, 0x5c)

# 5. Set Gyroscope ODR to 208Hz, 2000dps
imu.__write_reg(CTRL2_G, 0x5c)

# 6. Enable Timestamp routing to the FIFO at every Data-Ready (DRDY)
imu.__write_reg(FIFO_CTRL2, 0x80)

# 7. Set FIFO decimation: No decimation for Accel and Gyro (Datasets 1 & 2)
imu.__write_reg(FIFO_CTRL3, 0x09)

# 8. Set FIFO decimation: No decimation for Timestamp (Dataset 4)
imu.__write_reg(FIFO_CTRL4, 0x08)

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

imu_ready = False
imu_samples = bytearray()

start_ts = time.ticks_us()
# imu_start_ts = read_current_timestamp()
img_ts = time.ticks_us()

protocol.register(name="frame", backend=FrameChannel())
protocol.register(name="imu", backend=ImuChannel())

# reset timestamp count
imu.__write_reg(TIMESTAMP2_REG, 0xaa)
# bypass (FIFO_MODE=000) empties the FIFO and resets its pointers
imu.__write_reg(FIFO_CTRL5, 0x00)
time.sleep_ms(5)
imu.__write_reg(FIFO_CTRL5, 0x2E)

# prv_ts_raw = 0
while True:
    if not imu_ready:
        imu_samples = read_fifo_continuous_with_timestamp()
        # print(len(imu_samples))
        imu_ready = bool(imu_samples)
#    for imu_sample in imu_samples:
#        gx, gy, gz, ax, ay, az, ts_raw = imu_sample
#        print(ax, ay, az, (ts_raw - prv_ts_raw)*0.025)
#        prv_ts_raw = ts_raw
#    if not frame_ready and csi0.readable():
    if not frame_ready:
        now_ts = time.ticks_us()
        if time.ticks_diff(now_ts, img_ts) > 49000:
            img = csi0.snapshot()
            img_ts = now_ts
            img_mv = memoryview(img)
            frame_ready = True
