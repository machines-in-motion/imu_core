#ifndef IMU_GETTER_HPP
#define IMU_GETTER_HPP

#include <map>
#include <string>
#include <memory>

#include <cstdlib>
#include "ros/ros.h"
#include "ros/param.h"
#include "rosrt/rosrt.h"
#include "amd_clmc_ros_messages/Imu.h"
#include <map>
#include <memory>
#include <algorithm>
#include <string>


#ifdef __XENO__

#include <native/task.h>
#include <sys/mman.h>
#define REALTIME true

#else

#define REALTIME false

#endif

#define TOPIC "/IMU/"

class ImuData {
 public:
  double acceleration[3];
  double angular_rate[3];
  double timestamp;
  void print(std::string header);
};


class ImuDataGetter {
 public:
  virtual std::shared_ptr<ImuData> get()=0;
};

class ArrayImuDataGetter {
  
 public:
  void add(std::string id, std::shared_ptr<ImuDataGetter> imu_data_getter);
  void get(std::map< std::string , std::shared_ptr<ImuData> > &update); 
 private:
  std::map< std::string , std::shared_ptr<ImuDataGetter> > imu_data_getters;
  
};

class Basic_ImuDataGetter : public ImuDataGetter {
 public:
  public Basic_ImuDataGetter(std::string device,uint8_t message_type,bool is_45);
  std::shared_ptr<ImuData> get();
 private:
  std::shared_ptr<ImuData> imu_data;
  std::shared_ptr<ImuInterface> imu_interface;
}

std::shared_ptr<ArrayImuDataGetter> get_basic_imu_data_getter(std::vector<std::string> devices,uint8_t message_type,bool is_45);

