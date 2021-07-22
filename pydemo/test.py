import time
import signal
import sys
import numpy as np
import imu_core.imu_core_cpp as IMU


def handle_signal(s, f):
    # this way is better then the commented version because you want 
    # to force it to call destructor immediately
    # For example, in ipython, you would have to exit the ipython 
    # instead completely instead of just ctrl-c to exit the script
    print("\nExiting!\n")
    global imu
    del imu
    sys.exit(0)
    
signal.signal(signal.SIGINT, handle_signal)

argv = sys.argv

print(argv[0])

if len(argv) == 1:
    # default port
    port = "/dev/ttyACM1"
elif len(argv) == 2:
    # use command line argument
    port = argv[1]
else:
    # wrong number of arguments
    sys.exit(f"Wrong number of arguments: expected 1, received {len(argv)}")


imu = IMU.Imu3DM_GX3_25(port, True)

imu.initialize()

np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')
while True:
    print("Acceleration: ", imu.get_acceleration(), "Angular Rate: ", imu.get_angular_rate())
    time.sleep(0.05)
