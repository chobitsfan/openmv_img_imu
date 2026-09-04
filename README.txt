taskset -c 0 python host.py acc_cali_3.csv
taskset -c 1,2,3 ./scripts/monoVIO_openmv_ae3.bash
