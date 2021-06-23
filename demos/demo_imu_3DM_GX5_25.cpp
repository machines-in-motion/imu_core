/**
 * @file test.cpp
 * @author Vincent Berenz (vincent.brenz@tuebingen.mpg.de)
 * @brief Testing imu connection directly via the drivers. See test_interface in
 *        the same package for an example of the API.
 * @version 0.1
 * @date 2019-05-09
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "real_time_tools/timer.hpp"
#include "imu-core/imu_3DM_GX5_25.hpp"

using namespace std;

bool keep_running = true;

// Define the function to be called when ctrl-c (SIGINT) is sent to process.
void signal_callback_handler(int signum) {
   // Terminate program
   keep_running = false;
}

int main(int argc, char** argv){
  /**
   * Manage the arguments
   */
  if (argc <= 1){
    printf("usage: demo_imu_3DM_GX5_25 <device>\ne.g. demo_imu_3DM_GX5_25 /dev/ttyACM0\n");
    return -1;
  }
  std::string device = std::string(argv[1]);

  /**
   * create the imu object.
   */
  bool stream_data = true;
  imu_core::imu_3DM_GX5_25::Imu3DM_GX5_25 imu (device, stream_data);
  imu.initialize();

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  while (keep_running)
  {
    Eigen::Vector3d acc = imu.get_acceleration();
    Eigen::Vector3d ang_rate = imu.get_angular_rate();

    std::cout << "acc = [";

    for (int i = 0; i < 3; i++) {
      printf("%+0.3f ", acc(i));
    }

    std::cout << "]; ang rate = [";

    for (int i = 0; i < 3; i++) {
      printf("%+0.3f ", ang_rate(i));
    }

    std::cout << "];" << std::endl;

    real_time_tools::Timer::sleep_sec(0.05);
  }
  return 0;
}

