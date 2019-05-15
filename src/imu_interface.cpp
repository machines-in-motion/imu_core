/**
 * Generic interface for the 3DM-GX3-25 IMU for Xenomai and non xenomai machines.
*/

#include "imu-core/imu_interface.hpp"

namespace imu_core{

/**
 * Definition of ImuMsg class
 */

ImuMsg::ImuMsg()
{
  command_.clear();
  reply_.clear();
}

bool ImuMsg::is_valid()
{
  return (command_.size() > 0) && (reply_.size() > 0);
}

std::string ImuMsg::reply_debug_string()
{
  return real_time_tools::UsbStream::msg_debug_string(reply_);
}

std::string ImuMsg::command_debug_string()
{
  return real_time_tools::UsbStream::msg_debug_string(command_);
}

/**
 * Definition of ImuInterface class
 */

ImuInterface::ImuInterface(const std::string port_name)
{
  port_name_ = port_name;
}

}