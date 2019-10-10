#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include <iomanip>
#include <sstream>
#include <vector>
#include <Eigen/Eigen>
#include "real_time_tools/usb_stream.hpp"

namespace imu_core{

/**
 * @brief This class allow the user to write or read some msg.
 */
class ImuMsg
{
public:
  /**
   * @brief Construct a new ImuMsg object.
   */
  ImuMsg();
  /**
   * @brief Destroy the ImuMsg object.
   */
  ~ImuMsg(){}
  /**
   * @brief Check if the command_ and the reply_ objects has been initialized.
   */
  bool is_valid();
  /**
   * @brief Return the reply debug string
   * 
   * @return std::string 
   */
  std::string reply_debug_string();
  /**
   * @brief Return the command debug string
   * 
   * @return std::string 
   */
  std::string command_debug_string();
public:
  /**
   * @brief Contain the command to be written or that has been written
   */
  std::vector<uint8_t> command_;
  /**
   * @brief contain the device reply
   */
  std::vector<uint8_t> reply_;
};

/**
 * @brief This interface propose an easy acces to the IMU data though Eigen.
 */
class ImuInterface {
public:
  /**
   * @brief Construct a new ImuInterface object
   * 
   * @param port_name is the device path in linux: e.g. /dev/tty0
   * @param stream_data is if the user wants to stream the data or poll them.
   */
  ImuInterface(const std::string port_name);
  /**
   * @brief Destroy the ImuInterface object
   */
  virtual ~ImuInterface(void){}
  /**
   * @brief Initialization method that will depend on each IMU.
   * 
   * @return true if success.
   * @return false is failure.
   */
  virtual bool initialize() = 0;

  /**
   * @brief Get the linear acceleration of the imu.
   * 
   * @return const Eigen::Ref<Eigen::Vector3d> 
   */
  const Eigen::Ref<Eigen::Vector3d> get_acceleration()
  {
    return acceleration_;
  }
  /**
   * @brief Get the angular rate of the imu
   * 
   * @return const Eigen::Ref<Eigen::Vector3d> 
   */
  const Eigen::Ref<Eigen::Vector3d> get_angular_rate()
  {
    return angular_rate_;
  }
  /**
   * @brief Utilities to swap 16 bits
   * 
   * @param x 
   * @return uint16_t 
   */
  static inline uint16_t bswap_16(uint16_t x) {
    return (x>>8) | (x<<8);
  }
  
  /**
   * @brief Utilities to swap 32 bits
   * 
   * @param x 
   * @return uint32_t 
   */
  static inline uint32_t bswap_32(uint32_t x) {
    return (bswap_16(x&0xffff)<<16) | (bswap_16(x>>16));
  }

protected:
  /**
   * @brief Measurement of the accelerometer
   */
  Eigen::Vector3d acceleration_;
  /**
   * @brief Measurement of the gyroscope.
   */
  Eigen::Vector3d angular_rate_;
  /**
   * @brief Name of the linux path, e.g. /dev/ttyACM0
   */
  std::string port_name_;
};

} // namespace
#endif // Header protection
