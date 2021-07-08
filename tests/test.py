import time, signal, sys, numpy as np, imu_core.imu_core_cpp as IMU


np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')
keep_running = True

def handle_signal(s, f):
    global keep_running
    keep_running = False
    print("\nexiting")

signal.signal(signal.SIGINT, handle_signal)

imu = IMU.Imu3DM_GX3_25("/dev/ttyACM1", True)

imu.initialize()


while keep_running:
    print("Acceleration: ", imu.get_acceleration(), "Angular Rate: ", imu.get_angular_rate())
    time.sleep(0.05)

sys.exit(0)
