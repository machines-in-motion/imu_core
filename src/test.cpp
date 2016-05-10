

#include "imu-ros-rt/ImuInterface.h"

#define DEVICE "ttyACM0"
#define IS_45 false
#define NB_ITERATIONS 20
#define SLEEP 50000 // micro seconds
#define MESSAGE_TYPE 0xc8

#ifdef __XENO__

  #define REALTIME true

  static void print_imu_data(double &timestamp,double *accel, double* angrate){
    rt_printf("%f\t|\t%f\t%f\t%f\t|\t%f\t%f\t%f\n",timestamp,accel[0],accel[1],accel[2],angrate[0],angrate[1],angrate[2]);
  }

  static void sleep(){
    rt_task_sleep(SLEEP*1000); 
  }

#else

  #define REALTIME false

  static void print_imu_data(double &timestamp,double *accel, double* angrate){
    printf("%f\t|\t%f\t%f\t%f\t|\t%f\t%f\t%f\n",timestamp,accel[0],accel[1],accel[2],angrate[0],angrate[1],angrate[2]);
  }

  static void sleep(){
    usleep(SLEEP);
  }

#endif


int main(){

  uint8_t message_type[1];
  int num_messages = 1;
  message_type[0] = MESSAGE_TYPE;

  ImuInterface myIMU(DEVICE,true,REALTIME,IS_45);
  myIMU.initialize(message_type,num_messages);

  double accel[3];
  double angrate[3];
  double timestamp;

  for(int i=0;i<NB_ITERATIONS;i++){
    myIMU.readAccelAngrate(accel,angrate,timestamp);
    print_imu_data(timestamp,accel,angrate);
    sleep();
  }

  

}

