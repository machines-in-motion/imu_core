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

#define IS_45 true
#define NB_ITERATIONS 20
#define SLEEP 50 // milli seconds
#define MESSAGE_TYPE 0xc8

static void print_imu_data(double &timestamp,double *accel, double* angrate){
  rt_printf("%f\t|\t%f\t%f\t%f\t|\t%f\t%f\t%f\n",timestamp,accel[0],accel[1],accel[2],angrate[0],angrate[1],angrate[2]);
}

static void sleep(){
  real_time_tools::Timer::sleep_ms(SLEEP); 
}

int main(int argc, char** argv){
  
  if (argc <= 1){
    printf("usage: test <device>\ne.g. test tty0\n");
    return -1;
  }

  std::string device = std::string(argv[1]);

  uint8_t message_type[1];
  int num_messages = 0;
  message_type[0] = MESSAGE_TYPE;
  
  bool real_time = false;
  bool stream_data = true;

  Imu3DM_GX3_25 myIMU(device.c_str(), stream_data, real_time, IS_45);
  myIMU.initialize(message_type,num_messages);
  
  double accel[3];
  double angrate[3];
  double timestamp;
  
  // for(int i=0;i<NB_ITERATIONS;i++){
  //   myIMU.readAccelAngrate(accel,angrate,timestamp);
  //   print_imu_data(timestamp,accel,angrate);
  //   sleep();
  // }
  
  return 0;
}

