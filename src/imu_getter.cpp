#include "imu-ros-rt/imu_getter.h"


static inline void print_xyz(double *p){

  printf("%f\t%f\t%f",p[0],p[1],p[2]);

}


void ImuData::print(std::string header){

  printf("%s\t|\t",header.c_str());
  print_xyz(this->acceleration);
  printf("\t|\t");
  print_xyz(this->angular_rate);
  printf("\n");

}


void ArrayImuDataGetter::add(std::string id, std::shared_ptr<ImuDataGetter> imu_data_getter){

  this->imu_data_getters[id] = imu_data_getter;

}


void ArrayImuDataGetter::get(std::map< std::string , std::shared_ptr<ImuData> > &update){

  for( std::map<std::string,std::shared_ptr<ImuDataGetter> >::iterator iter = this->imu_data_getters.begin(); iter != this->imu_data_getters.end(); ++iter){
    std::string id =  iter->first;
    std::shared_ptr<ImuDataGetter> getter = iter->second;
    update[id] = getter->get();
  }

}


Basic_ImuDataGetter::Basic_ImuDataGetter(std::string device,uint8_t message_type,bool is_45){

  uint8_t message_type_a[1];
  int num_messages = 1;
  message_type_a[0] = message_type;

  this->imu_data.reset(new ImuData());
  this->imu_interface.reset(new ImuInterface(device),true,REALTIME,is_45);
  this->imu_interface.initialize(message_type,num_messages);

}

std::shared_ptr<ImuData> Basic_ImuDataGetter::get(){

  this->imu_interface.readAccelAngrate(this->imu_data->acceleration,this->imu_data->angular_rate,this->imu_data->timestamp);
  return this->imu_data;

}


std::shared_ptr<ArrayImuDataGetter> get_basic_imu_data_getter(std::vector<std::string> devices,uint8_t message_type,bool is_45){

  std::shared_ptr<ArrayImuDataGetter> array_data_getter(new ArrayImuDataGetter());
  for(int i=0;i<devices.size();i++){
    std::shared_ptr<RosRT_ImuDataGetter> data_getter(new Basic_ImuDataGetter(devices[i],message_type,is_45));
    array_data_getter->add(devices[i],data_getter);
  }
  return array_data_getter;

}

