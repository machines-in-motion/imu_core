import imu_core.imu_core_cpp, time, numpy as np, signal, sys


def handle_signal(s, f):
    global keep_running
    keep_running = False


keep_running = True
imu = imu_core.imu_core_cpp.Imu3DM_GX3_25("/dev/ttyACM1", True)  # assuming port of /dev/ttyACM1

# imu.reset_device()
imu.initialize()
signal.signal(signal.SIGINT, handle_signal)
while keep_running:
    # print("acc: ", [f"{e:+.3f}" for e in imu.get_acceleration()], "ang rate: ", [f"{e:+.3f}" for e in imu.get_angular_rate()])
    print("acc = [", end='')

    for e in imu.get_acceleration():
        print(f"{e:+.3f} ", end='')

    print("]  ang rate = [", end='')

    for e in imu.get_angular_rate():
        print(f"{e:+.3f} ", end='')

    print("]")

    time.sleep(0.05)

del imu  # explicitly call destructor
sys.exit(0)
