import signal
import sys
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True, sign=' ', floatmode='fixed')
import imu_core.imu_core_cpp as IMU


if __name__ == "__main__":
    def handle_signal(s, f):
        global run
        run = False
        
    signal.signal(signal.SIGINT, handle_signal)

    if len(sys.argv) == 1:
        # default port
        # port = "/dev/ttyACM0"
        port = "/dev/ttyACM1"
    elif len(sys.argv) == 2:
        # use command line argument
        port = sys.argv[1]
    else:
        # wrong number of arguments
        sys.exit(f"Wrong number of arguments: expected 1, received {len(sys.argv) - 1}: {sys.argv[1:]}")


    imu = IMU.Imu3DM_GX3_25(port, True)

    imu.initialize()

    run = True

    while run:
        print("Acceleration: ", imu.get_acceleration(), "Angular Rate: ", imu.get_angular_rate())
        time.sleep(0.05)

    print("\nExiting!")    
    # C++ destructor will get automatically called, but not necessarily when the 
    # script ends.
    # This can lead to problems when using ipython if you don't exit out of ipython
    # fully before calling this script again.
    # Eg., if you run ipython, run test.py, ctrl-c, the destructor won't run. 
    # This is because ipython keeps variables.
    # del imu forces the destructor to run.
    del imu
    sys.exit(0)