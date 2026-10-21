Scripts for IMU and camera synchronization on the OpenMV AE3.
It requires openmv-python(https://github.com/openmv/openmv-python) and an OpenMV AE3 with modified firmware(https://github.com/chobitsfan/openmv/tree/kimera-vio)
Usage: python host.py acc_cali.csv
acc_cali.csv is the accelerometer calibration file contains x, y, z offset and scale, You can get these values from accel_cali(https://github.com/chobitsfan/accel_cali). Omit the calibration file name when calibrating.
Use taskset to arrange kimera-vio or other heavy job to different CPU core to avoid interference
