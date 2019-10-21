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

  for(unsigned i=0 ; i<100 ; ++i)
  {
    std::cout << "acc = " << imu.get_acceleration().transpose()
              << " ; ang rate = " << imu.get_angular_rate().transpose()
              << std::endl;
    real_time_tools::Timer::sleep_ms(5);
  }
  return 0;
}

