/**
 * @file reset_3DM_GX3_25.cpp
 * @author Maximilien Naveau
 * @brief Reset the IMU with the simplest code.
 * @version 0.1
 * @date 2019-05-09
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "real_time_tools/timer.hpp"
#include "imu-core/imu_3DM_GX3_25.hpp"

int main(int argc, char** argv){
  /**
   * Manage the arguments
   */
  if (argc <= 1){
    printf("usage: test <device>\ne.g. test tty0\n");
    return -1;
  }
  std::string device = std::string(argv[1]);

  /**
   * create the imu object.
   */
  bool stream_data = false;
  imu_core::imu_3DM_GX3_25::Imu3DM_GX3_25 imu (device, stream_data);
  imu.open_usb_port();
  imu.reset_device();
  return 0;
}

