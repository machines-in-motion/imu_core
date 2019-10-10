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
  bool stream_data = true;
  imu_core::imu_3DM_GX3_25::Imu3DM_GX3_25 imu (device, stream_data);
  imu.initialize();

  for(unsigned i=0 ; i<20 ; ++i)
  {
    std::cout << "acc = " << imu.get_acceleration().transpose()
              << " ; ang rate = " << imu.get_angular_rate().transpose()
              << std::endl;
  }
  return 0;
}

