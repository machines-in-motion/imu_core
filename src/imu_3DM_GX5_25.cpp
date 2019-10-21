/**
 * @file imu_3DM_GX5_25.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-09-13
 * 
 * Generic interface for the 3DM-GX5-25 IMU
 */

#include <real_time_tools/spinner.hpp>
#include "imu-core/imu_3DM_GX5_25.hpp"

using namespace real_time_tools;
namespace imu_core{
namespace imu_3DM_GX5_25{

Imu3DM_GX5_25::Imu3DM_GX5_25(const std::string& port_name,
                             const bool& stream_data):
  ImuInterface(port_name)
{
  /** Nothing to be done for these attributes
  thread_;
  mutex_;
  usb_stream_;
  */
  // We try 3 times maximum before giving up on realigning.
  max_realign_trials_ = 3;
  timer_.set_memory_size(100000); // 100 seconds at 1ms/iteration
  time_stamp_ = 0.0;
  stream_data_ = stream_data;
  stop_imu_communication_ = false;
  aligned_data_ = false;
}

Imu3DM_GX5_25::~Imu3DM_GX5_25(void)
{
  stop_reading_loop();
  while(!idle_mode()){};  
  usb_stream_.close_device();
}

bool Imu3DM_GX5_25::initialize()
{
  // open OS communication
  bool initialized = open_usb_port();
  if(!initialized)
  {
    throw std::runtime_error("Cannot open device: " + port_name_);
  }
  
  // setup the IMU configuration
  while(!idle_mode()){};
  while(!imu_data_1kHz()){};
  // while(!estimation_filter_data_500Hz()){};
  while(!set_heading_at_0()){};
  while(!start_streaming_data()){};
  if(DEBUG_PRINT_IMU_GX5_25)
  {
    std::cout << "Initialized? : " << (initialized? "true":"false") << std::endl;
  }
  if(initialized)
  {
    thread_.create_realtime_thread(Imu3DM_GX5_25::reading_loop_helper, this);
  }
  wait_data_aligned();
  return true;
}

bool Imu3DM_GX5_25::open_usb_port(int bauderate)
{
  // open OS communication
  bool sucess = usb_stream_.open_device(port_name_);
  // setup OS communication
  port_config_.rts_cts_enabled_ = false;
  port_config_.parity_ = false;
  port_config_.stop_bits_ = real_time_tools::PortConfig::StopBits::one;
  port_config_.prepare_size_definition_ = false;
  port_config_.data_bits_ = real_time_tools::PortConfig::cs8;
  port_config_.baude_rate_ = bauderate;
  usb_stream_.set_port_config(port_config_);
  usb_stream_.set_poll_mode_timeout(1.0);
  usb_stream_.flush();
  return sucess;
}


bool Imu3DM_GX5_25::receive_message(ImuMsg& msg, bool stream_mode)
{
  if(!usb_stream_.read_device(msg.reply_, stream_mode))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::receive_message(): [Error] "
                "message badly received\n");
    }
  }

  if (!is_checksum_correct(msg))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::receive_message(): [Warning] "
              "Received message with bad checksum: %s\n",
              msg.reply_debug_string().c_str());
    }
    if (stream_mode)
    {
      if(DEBUG_PRINT_IMU_GX5_25)
      {
        rt_printf("Imu3DM_GX5_25::receive_message(): [Status] "
                  "Attempting to re-align with stream...\n");
      }
      return read_misaligned_msg_from_device(msg);
    }
    return false;
  }
  else if (msg.reply_[0] != msg.command_[0] || msg.reply_[1] != msg.command_[1])
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::receive_message(): [Error] "
              "Received unexpected message from device.\n");
    }
    return false;
  }
  return true;
}

bool Imu3DM_GX5_25::is_checksum_correct(const ImuMsg& msg)
{
  // First we compute the sum of the bytes of the response message except the
  // last 2 bytes. The last 2 bytes should be the sum of all the other bytes.
  uint8_t checksum_byte1 = 0;
  uint8_t checksum_byte2 = 0;
  GX5ImuMsg::compute_checksum(msg.reply_, checksum_byte1, checksum_byte2); 
  // return the test value.
  return (msg.reply_[msg.reply_.size() - 2] == checksum_byte1 &&
          msg.reply_[msg.reply_.size() - 1] == checksum_byte2);
}

bool Imu3DM_GX5_25::read_misaligned_msg_from_device(ImuMsg& msg)
{
  // When we read corrupt data, try to keep reading until we catch up with 
  // clean data:
  int trial = 0;
  while ((msg.reply_[0] != msg.command_[0]) ||
         (msg.reply_[1] != msg.command_[1]) ||
         (msg.reply_[2] != msg.command_[2]) || !is_checksum_correct(msg))
  {
    // If to many tries we give up
    if (trial >= max_realign_trials_)
    {
      if(DEBUG_PRINT_IMU_GX5_25)
      {
        rt_printf("Imu3DM_GX5_25::read_misaligned_msg_from_device(): [Error] "
                  "Realigning failed!\n");
      }
      return false;
    }
    // Print the corrupt message:
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::read_misaligned_msg_from_device(): [Warning] "
                "Read invalid message: %s\n",
                msg.reply_debug_string().c_str());
    }
    // Search for the header:
    unsigned int num_missed = 1;
    for (; num_missed < msg.reply_.size() - 2; ++num_missed)
    {
      if ( (msg.command_[0] = msg.reply_[num_missed + 0]) &&
           (msg.command_[1] = msg.reply_[num_missed + 1]) &&
           (msg.command_[2] = msg.reply_[num_missed + 2]) )
      {
        break;
      }
    }
    // No header found
    if (num_missed >= msg.reply_.size())
    {
      if(DEBUG_PRINT_IMU_GX5_25)
      {
        rt_printf("Imu3DM_GX5_25::read_misaligned_msg_from_device(): [Error] "
                  "Realigning failed!\n");
      }
      return false;
    }
    // We MIGHT have found the header!
    std::vector<uint8_t> fragment (num_missed, 0);
    // We read the rest of the message
    bool stream_on = true; // simple read
    if(usb_stream_.read_device(fragment, stream_on))
    {
      if(DEBUG_PRINT_IMU_GX5_25)
      {
        rt_printf("Imu3DM_GX5_25::read_misaligned_msg_from_device(): [Error] "
                  "Could not read fragment.\n");
      }
      return false;
    }
    // we shift the end of teh message to the beginning
    unsigned start_index = msg.reply_.size()-num_missed;
    for(unsigned i=0 ; i<start_index ; ++i)
    {
      msg.reply_[i] = msg.reply_[i+start_index];
    }
    // we copy the rest of the message in the msg.reply_
    for(unsigned i=start_index ; i<msg.reply_.size() ; ++i)
    {
      msg.reply_[i] = fragment[i-start_index];
    }
    // in case all the procedure failed we try again.
    ++trial;
  }

  return true;
}

bool Imu3DM_GX5_25::idle_mode()
{
  usb_stream_.flush();
  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::idle_mode(): [Status] sending...\n");
  }

  // send the configuration to the IMU
  IdleModeMsg msg;
  if (!send_message(msg))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::idle_mode(): [Error] sending message failed\n");
    }
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::idle_mode(): [Error] receiving answer failed\n");
    }
    return false;
  }

  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::idle_mode(): [Status] sucess with reply: %s\n",
          msg.reply_debug_string().c_str());
  }
  return true;
}
bool Imu3DM_GX5_25::imu_data_1kHz(){
  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::imu_data_1kHz(): [Status] sending...\n");
  }

  // send the configuration to the IMU
  AccGyr1kHzMsg msg;
  if (!send_message(msg))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::imu_data_1kHz(): [Error] sending message failed\n");
    }
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::imu_data_1kHz(): [Error] receiving answer failed\n");
    }
    return false;
  }

  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::imu_data_1kHz(): [Status] sucess with reply: %s\n",
          msg.reply_debug_string().c_str());
  }
  return true;
}

bool Imu3DM_GX5_25::estimation_filter_data_500Hz()
{
  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::estimation_filter_data_500Hz(): [Status] sending...\n");
  }

  // send the configuration to the IMU
  EFdata500HzMsg msg;
  if (!send_message(msg))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::estimation_filter_data_500Hz(): [Error] sending message failed\n");
    }
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::estimation_filter_data_500Hz(): [Error] receiving answer failed\n");
    }
    return false;
  }

  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::estimation_filter_data_500Hz(): [Status] sucess with reply: %s\n",
          msg.reply_debug_string().c_str());
  }
  return true;
}

bool Imu3DM_GX5_25::start_streaming_data(void)
{
  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::start_streaming_data(): [Status] sending...\n");
  }

  // send the configuration to the IMU
  StreamImuEfMsg msg;
  if (!send_message(msg))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::start_streaming_data(): [Error] sending message failed\n");
    }
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::start_streaming_data(): [Error] receiving answer failed\n");
    }
    return false;
  }

  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::start_streaming_data(): [Status] success with reply: %s\n",
          msg.reply_debug_string().c_str());
  }
  return true;
}

bool Imu3DM_GX5_25::set_heading_at_0(){
  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::set_heading_at_0(): [Status] sending...\n");
  }

  // send the configuration to the IMU
  SetHeading0Msg msg;
  if (!send_message(msg))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::set_heading_at_0(): [Error] sending message failed\n");
    }
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::set_heading_at_0(): [Error] receiving answer failed\n");
    }
    return false;
  }

  if(DEBUG_PRINT_IMU_GX5_25)
  {
    rt_printf("Imu3DM_GX5_25::set_heading_at_0(): [Status] success with reply: %s\n",
          msg.reply_debug_string().c_str());
  }
  return true;
}

bool Imu3DM_GX5_25::reading_loop(void)
{
  real_time_tools::Spinner spinner;
  spinner.set_period(0.002);
  usb_stream_.flush();
  while (!stop_imu_communication_)
  {
    spinner.spin();
    aligned_data_ = receive_data(stream_data_);
  }
  return true;
}

bool Imu3DM_GX5_25::stop_reading_loop(void)
{
  mutex_.lock();
  stop_imu_communication_ = true;
  mutex_.unlock();
  thread_.join();
  return true;
}

bool Imu3DM_GX5_25::receive_data(bool)
{
  if(!usb_stream_.read_device(imu_data_msg_.reply_, true))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::receive_data(): [Error] message badly received\n");
    }
    return false;
  }

  if (!is_checksum_correct(imu_data_msg_))
  {
    if(DEBUG_PRINT_IMU_GX5_25)
    {
      rt_printf("Imu3DM_GX5_25::receive_message(): [Warning] "
              "Received message with bad checksum: %s\n",
              imu_data_msg_.reply_debug_string().c_str());
      rt_printf("Imu3DM_GX5_25::receive_message(): [Status] "
                "Attempting to re-align with stream...\n");
    }
    if(!read_misaligned_msg_from_device(imu_data_msg_))
    {
      return false;
    }
  }
  // if (!receive_message(ef_data_msg_, stream_data))
  // {
  //   rt_printf("Imu3DM_GX5_25::receive_data(): [Error] error collectiong estimation filter data\n");
  //   return false;
  // }

  mutex_.lock();
  
  acceleration_[0] = double_from_byte_array(imu_data_msg_.reply_, index_acc_x);
  acceleration_[1] = double_from_byte_array(imu_data_msg_.reply_, index_acc_y);
  acceleration_[2] = double_from_byte_array(imu_data_msg_.reply_, index_acc_z);

  angular_rate_[0] = double_from_byte_array(imu_data_msg_.reply_, index_gyro_x);
  angular_rate_[1] = double_from_byte_array(imu_data_msg_.reply_, index_gyro_y);
  angular_rate_[2] = double_from_byte_array(imu_data_msg_.reply_, index_gyro_z);
  
  // rpy_[0] = double_from_byte_array(ef_data_msg_.reply_, index_rpy_x);
  // rpy_[1] = double_from_byte_array(ef_data_msg_.reply_, index_rpy_y);
  // rpy_[2] = double_from_byte_array(ef_data_msg_.reply_, index_rpy_z);

  mutex_.unlock();
  return true;
}

} // namespace imu_3DM_GX3_25
} // namespace imu_core
