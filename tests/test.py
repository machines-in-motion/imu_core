# WIP, just unplug and replug in imu if errors occur when exiting process

import imu_core.imu_core_cpp, time, numpy as np, signal, sys

np.set_printoptions(precision=3)

imu = imu_core.imu_core_cpp.Imu3DM_GX3_25("/dev/ttyACM1", True)  # assuming port of /dev/ttyACM1

imu.initialize()

def handle_signal(signal_, frame_):
    #   TODO
    #   stop_reading_loop();
    #   while(!stop_streaming_data()){}
    #   reset_device();
    #   usb_stream_.close_device();
    sys.exit(0)

signal.signal(signal.SIGINT, handle_signal)

while True:
    print("Accleration: ", imu.get_acceleration(), "Angular Rate: ", imu.get_angular_rate())
    time.sleep(0.05)
