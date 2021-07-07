import imu_core.imu_core_cpp, time, signal, sys
# import numpy as np
import numpy as np
np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')

keep_running = True

def handle_signal(s, f):
    global keep_running
    keep_running = False
    print("\nexiting")

signal.signal(signal.SIGINT, handle_signal)

imu = imu_core.imu_core_cpp.Imu3DM_GX3_25("/dev/ttyACM1", True)  # assuming port of /dev/ttyACM1
imu.initialize()




# del imu 

# imu = imu_core.imu_core_cpp.Imu3DM_GX3_25("/dev/ttyACM1", True)  # assuming port of /dev/ttyACM1
# imu.initialize()


while keep_running:
    print("Acceleration: ", imu.get_acceleration(), "Angular Rate: ", imu.get_angular_rate())
    # print("acc: ", [f"{e:+.3f}" for e in imu.get_acceleration()], "ang rate: ", [f"{e:+.3f}" for e in imu.get_angular_rate()])
    # print("acc = [", end='')

    # for e in imu.get_acceleration():
    #     print(f"{e:+.3f} ", end='')

    # print("]  ang rate = [", end='')

    # for e in imu.get_angular_rate():
    #     print(f"{e:+.3f} ", end='')

    # print("]")

    time.sleep(0.05)

del imu  # explicitly call destructor
sys.exit(0)

# import imu_core.imu_core_cpp, time, signal, sys

# imu = imu_core.imu_core_cpp.Imu3DM_GX3_25("/dev/ttyACM1", True)  # assuming port of /dev/ttyACM1

# imu.initialize()

# import numpy as np  # can't import before imu or else it breaks 
# np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')

# def handle_signal(signal_, frame_):
#     sys.exit(0)

# signal.signal(signal.SIGINT, handle_signal)

# while True:
#     print("Acceleration: ", imu.get_acceleration(), "Angular Rate: ", imu.get_angular_rate())
#     time.sleep(0.05)