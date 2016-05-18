#include "imu-core/ImuInterface.h"

/*
  testing imu connection via the getter, which is the easiest preferred solution.
  pass devices (e.g tty0) as argument
 */

#define IS_45 false
#define NB_ITERATIONS 20
#define SLEEP 50000 // micro seconds
#define MESSAGE_TYPE 0xc8

int main(int argc, char** argv){
  
  if (argc!=1){

    printf("usage: test_interface <devices>\ne.g. test_interface tty0 tty1\n");

  } else {

    std::vector<std::string> devices;
    for(int i=0;i<argc;i++){
      devices.push_back(std::string(argv[i]));
    }

    std::shared_ptr<ArrayImuDataGetter> imu_data_getter = get_basic_imu_data_getter(devices,MESSAGE_TYPE,IS_45);
    std::map< std::string , std::shared_ptr<ImuData> > data;    

    for(int i=0;i<NB_ITERATIONS;i++){
      imu_data_getter.get(data);
      for (int j=0;j<devices.size();j++){
	data[devices[j]].print();
      }
      usleep(SLEEP);
    }


  }
  

}

