import csi
import protocol
import time
import imu

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

# 1. Enable Block Data Update (BDU) and Auto-increment (IF_INC)
# BDU (bit 6) = 1, IF_INC (bit 2) = 1 -> 0x44
imu.__write_reg(CTRL3_C, 0x44)

# 4. Set FIFO decimation: No decimation for Accel and Gyro
# DEC_FIFO_GYRO = 001, DEC_FIFO_XL = 001 -> 0x09
imu.__write_reg(FIFO_CTRL3, 0x09)

# 5. Set FIFO ODR to 52Hz and mode to Continuous
# ODR_FIFO = 0011 (52Hz), FIFO_MODE = 110 (Continuous) -> 0x1e
# Continuous Mode: If the FIFO is full, new samples overwrite the oldest.
imu.__write_reg(FIFO_CTRL5, 0x1e)

# 2. Enable the Hardware Timestamp Timer
# TIMER_EN (bit 5) = 1 -> 0x20
imu.__write_reg(CTRL10_C, 0x20)

# 5. Enable Timestamp routing to the FIFO
# TIMER_PEDO_FIFO_EN (bit 7) = 1, TIMER_PEDO_FIFO_DRDY (bit 6) = 0 (Internal trigger)
imu.__write_reg(FIFO_CTRL2, 0x80)

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

    # DIFF_FIFO is an 11-bit value representing unread 16-bit words
    unread_words = status1 | ((status2 & 0x07) << 8)

    # With Gyro, Accel, and Timestamp enabled, each complete sample set is 9 words long:
    # - 3 words (6 bytes) for Gyro
    # - 3 words (6 bytes) for Accel
    # - 3 words (6 bytes) for the Timestamp dataset
    sample_sets = unread_words // 9

    data = []
    for _ in range(sample_sets):
        # 1. Read Gyroscope
        gx = read_fifo_word()
        gy = read_fifo_word()
        gz = read_fifo_word()

        # 2. Read Accelerometer
        ax = read_fifo_word()
        ay = read_fifo_word()
        az = read_fifo_word()

        # 3. Read Timestamp Dataset (3 words)
        ts_word0 = read_fifo_word_unsigned()
        ts_word1 = read_fifo_word_unsigned()
        read_fifo_word_unsigned()  # Must be read to clear the FIFO slot

        # The LSM6DSM 24-bit timestamp maps across the first two words:
        # Word 0: Timestamp bits [15:0]
        # Word 1: Timestamp bits [23:16] in the lower byte (bits [7:0])
        timestamp_24bit = ts_word0 | ((ts_word1 & 0x00FF) << 16)

        data.append((gx, gy, gz, ax, ay, az, timestamp_24bit))

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


protocol.register(name="frame", backend=FrameChannel())

while True:
    imu_data = read_fifo_continuous_with_timestamp()
    if len(imu_data) > 0:
        print(len(imu_data))
    if not frame_ready:
        img = csi0.snapshot()
        img_ts = time.ticks_us()
        img_mv = memoryview(img.bytearray())
        frame_ready = True
