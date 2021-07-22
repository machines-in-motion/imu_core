/**
 * Generic interface for the 3DM-GX3-25 IMU for Xenomai and non xenomai machines.
*/

#include "imu-core/imu_3DM_GX3_25.hpp"

using namespace real_time_tools;
namespace imu_core{
namespace imu_3DM_GX3_25{

Imu3DM_GX3_25::Imu3DM_GX3_25(const std::string& port_name,
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
}

Imu3DM_GX3_25::~Imu3DM_GX3_25(void)
{
  stop_reading_loop();
  while(!stop_streaming_data()){}
  reset_device();
  usb_stream_.close_device();
}

bool Imu3DM_GX3_25::initialize()
{
  // open OS communication
  bool initialized = open_usb_port();

  // setup the IMU configuration

  // communication_settings
  while(!set_communication_settings()){}
  while(!set_sampling_settings()){}
  while(!initialize_time_stamp()){}
  while(!capture_gyro_bias()){}
  while(!start_streaming_data()){}
  std::cout << "Initialized? : " << (initialized? "true":"false") << std::endl;
  if(initialized)
  {
    thread_.create_realtime_thread(Imu3DM_GX3_25::reading_loop_helper, this);
  }
  real_time_tools::Timer::sleep_ms(5);
  return true;
}

bool Imu3DM_GX3_25::open_usb_port(int bauderate)
{
  // open OS communication
  bool success = usb_stream_.open_device(port_name_);
  // setup OS communication
  port_config_.rts_cts_enabled_ = false;
  port_config_.parity_ = false;
  port_config_.stop_bits_ = real_time_tools::PortConfig::StopBits::one;
  port_config_.prepare_size_definition_ = false;
  port_config_.data_bits_ = real_time_tools::PortConfig::cs8;
  port_config_.baude_rate_ = bauderate;
  usb_stream_.set_port_config(port_config_);
  usb_stream_.set_poll_mode_timeout(0.1);
  // usb_stream_.flush();
  return success;
}


bool Imu3DM_GX3_25::receive_message(ImuMsg& msg, bool stream_mode)
{
  ssize_t read_bytes = usb_stream_.read_maybe_device(msg.reply_, stream_mode);
  if(read_bytes < 0)
  {
    rt_printf("Imu3DM_GX3_25::receive_message(): [Error] "
              "message badly received\n");
  }

  // rt_printf("\nBREAK debug received message of size %ld, expected %ld\n\n", asdf, msg.reply_.size());
  if(read_bytes != static_cast<ssize_t>(msg.reply_.size()))
  { 
    rt_printf("Imu3DM_GX3_25::receive_message(): [Error] "
              "Not enough bytes read. trying again... Requested %ld bytes and received %ld bytes.\n",
              msg.reply_.size(),
              read_bytes
              // msg_debug_string(msg.reply_, msg.reply_.size()).c_str()
              );
    // we didn't read enough bytes. try again
    usb_stream_.read_maybe_device(msg.reply_, stream_mode, read_bytes);
  }

  if (!is_checksum_correct(msg))
  {
    rt_printf("Imu3DM_GX3_25::receive_message(): [Warning] "
              "Received message with bad checksum: %s\n",
              msg.reply_debug_string().c_str());
    if (stream_mode)
    {
      rt_printf("Imu3DM_GX3_25::receive_message(): [Status] "
                "Attempting to re-align with stream...\n");
      return read_misaligned_msg_from_device(msg);
    }
    return false;
  }
  else if (msg.reply_[0] != msg.command_[0])
  {
    rt_printf("Imu3DM_GX3_25::receive_message(): [Error] "
              "Received unexpected message from device.\n");
    return false;
  }
  return true;
}

bool Imu3DM_GX3_25::is_checksum_correct(const ImuMsg& msg)
{
  // First we compute the sum of the bytes of the response message except the
  // last 2 bytes. The last 2 bytes should be the sum of all the other bytes.
  uint16_t data_checksum = 0;
  for (unsigned int i=0 ; i<msg.reply_.size()-2 ; ++i)
  {
    data_checksum += msg.reply_[i];
  }
  // Then we compute the big-endian of the last two bytes of the reply.
  uint16_t last_byte_checksum =
    bswap_16(*(uint16_t *) (&msg.reply_[msg.reply_.size()-2]) );
  // return the test value.
  return data_checksum == last_byte_checksum;
}

bool Imu3DM_GX3_25::read_misaligned_msg_from_device(ImuMsg& msg)
{

  // When we read corrupt data, try to keep reading until we catch up with
  // clean data:
  int trial = 0;
  while (msg.reply_[0] != msg.command_[0] || !is_checksum_correct(msg))
  {
    // If to many tries we give up
    if (trial >= max_realign_trials_)
    {
      rt_printf("Imu3DM_GX3_25::read_misaligned_msg_from_device(): [Error] "
                "Realigning failed!\n");
      return false;
    }
    // Print the corrupt message:
    rt_printf("Imu3DM_GX3_25::read_misaligned_msg_from_device(): [Warning] "
              "Read invalid message: %s\n",
              msg.reply_debug_string().c_str());
    // Search for the header:
    std::size_t num_missed = 1;
    for (; num_missed < msg.reply_.size(); ++num_missed)
    {
      if (msg.command_[0] == msg.reply_[num_missed])
      {
        break;
      }
    }
    // No header found
    if (num_missed >= msg.reply_.size())
    {
      rt_printf("Imu3DM_GX3_25::read_misaligned_msg_from_device(): [Error] "
                "Realigning failed!\n");
      return false;
    }
    // We MIGHT have found the header!
    std::vector<uint8_t> fragment (num_missed, 0);
    // We read the rest of the message
    bool stream_on = true; // simple read
    if(usb_stream_.read_device(fragment, stream_on))
    {
      rt_printf("Imu3DM_GX3_25::read_misaligned_msg_from_device(): [Error] "
                "Could not read fragment.\n");
      return false;
    }
    // we shift the end of the message to the beginning
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

bool Imu3DM_GX3_25::reset_device(void)
{
  rt_printf("Imu3DM_GX3_25::reset_device(): [Status] "
            "reset the device...\n");

  // send the configuration to the IMU
  ResetMsg msg;
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::reset_device(): [Error] "
              "Failed to send the reset the device message\n");
    return false;
  }

  rt_printf("Imu3DM_GX3_25::reset_device(): [Status] "
            "reset the device successfully\n");

  return true;
}


bool Imu3DM_GX3_25::set_communication_settings(void)
{
  rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Status] "
            "Setting communication settings...\n");

  // send the configuration to the IMU
  int bauderate = 921600;
  CommunicationSettingsMsg msg(bauderate);
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Error] "
              "Failed to send the communication settings message \n"
              "cmd: %s\nreply: %s\n", msg.command_debug_string().c_str(),
              msg.reply_debug_string().c_str());
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Error] "
              "Failed to receive the communication settings message reply.\n"
              "cmd: %s\nreply: %s\n", msg.command_debug_string().c_str(),
              msg.reply_debug_string().c_str());
    return false;
  }

  // reset the computer port Baude Rate
  port_config_.baude_rate_ = bauderate;
  usb_stream_.set_port_config(port_config_);

  assert(msg.get_baude_rate() == bauderate);

  rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Status] "
            "Set communication settings successfully.\n"
            "cmd: %s\nreply: %s\n", msg.command_debug_string().c_str(),
            msg.reply_debug_string().c_str());
  return true;
}

bool Imu3DM_GX3_25::set_sampling_settings(void)
{
  rt_printf("Imu3DM_GX3_25::set_sampling_settings(): [Status] "
            "Setting sampling settings...\n");

  // send the configuration to the IMU
  SamplingSettingsMsg msg;
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::set_sampling_settings(): [Error] "
              "Failed to send the sampling settings message\n");
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::set_sampling_settings(): [Error] "
              "Failed to receive the sampling settings message reply. "
              "Command: %s, Reply %s\n",msg.command_debug_string().c_str(),
              msg.reply_debug_string().c_str());
    return false;
  }

  rt_printf("Imu3DM_GX3_25::set_sampling_settings(): [Status] "
            "Set sampling settings successfully with reply: %s\n",
            msg.reply_debug_string().c_str());
  return true;
}

bool Imu3DM_GX3_25::initialize_time_stamp(void)
{
  rt_printf("Imu3DM_GX3_25::initialize_time_stamp(): [Status] "
            "Setting time stamp...\n");

  // send the configuration to the IMU
  TimerMsg msg;
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::initialize_time_stamp(): [Error] "
              "Failed to send the time stamp message\n");
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::initialize_time_stamp(): [Error] "
              "Failed to receive the time stamp message reply\n");
    return false;
  }

  rt_printf("Imu3DM_GX3_25::initialize_time_stamp(): [Status] "
            "Set time stamp successfully with reply: %s\n",
            msg.reply_debug_string().c_str());
  return true;
}

bool Imu3DM_GX3_25::capture_gyro_bias(void)
{
  rt_printf("Imu3DM_GX3_25::capture_gyro_bias(): [Status] "
            "Setting gyroscope bias...\n");

  // send the configuration to the IMU
  uint16_t calibration_duration = 3;
  CaptureGyroBiasMsg msg(calibration_duration);
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::capture_gyro_bias(): [Error] "
              "Failed to send the gyroscope bias message\n");
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = true; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::capture_gyro_bias(): [Error] "
              "Failed to receive the gyroscope bias message reply\n");
    return false;
  }

  rt_printf("Imu3DM_GX3_25::capture_gyro_bias(): [Status] "
            "Set gyroscope bias successfully with reply: %s\n",
            msg.reply_debug_string().c_str());
  return true;
}

bool Imu3DM_GX3_25::start_streaming_data(void)
{
  rt_printf("Imu3DM_GX3_25::start_streaming_data(): [Status] "
            "Setting continuous mode (start streaming data)...\n");

  // send the configuration to the IMU
  StartDataStreamMsg msg;
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::start_streaming_data(): [Error] "
              "Failed to send the continuous mode (start streaming data) "
              "message\n");
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = true; // Poll mode
  if (!receive_message(msg, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::start_streaming_data(): [Error] "
              "Failed to receive the continuous mode (start streaming data) "
              "message reply\n");
    return false;
  }

  rt_printf("Imu3DM_GX3_25::start_streaming_data(): [Status] "
            "Set continuous mode (start streaming data) successfully with "
            "reply: %s\n",
            msg.reply_debug_string().c_str());
  return true;
}

bool Imu3DM_GX3_25::stop_streaming_data(void)
{
  rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Status] "
            "Setting continuous mode (stop streaming data)...\n");

  // send the configuration to the IMU
  StopDataStreamMsg msg;
  if (!send_message(msg))
  {
    rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Error] "
              "Failed to send the continuous mode (stop streaming data) "
              "message\n");
    return false;
  }

  // Check that the Device received the message.
  bool stream_mode = true; // Poll mode
  if(msg.reply_.size()>2)
  {
    if (!receive_message(msg, stream_mode))
    {
      rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Error] "
                "Failed to receive the continuous mode (stop streaming data) "
                "message reply\n");
      return false;
    }
  }

  rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Status] "
            "Set continuous mode (stop streaming data) successfully with "
            "reply: %s\n",
            msg.reply_debug_string().c_str());
  return true;
}


bool Imu3DM_GX3_25::reading_loop(void)
{
  bool ret = true;
  while (!stop_imu_communication_ && ret)
  {
    // for (int i = 0; i < 1/** num_messages_*/ ; ++i)
    // {
      uint8_t data_type = DataType::AccGyro;
      switch (/**message_type_[i]*/DataType::AccGyro)
      {
      case DataType::AccGyro:
        ret = receive_acc_gyro(stream_data_);
        break;
      case DataType::StabAccGyroMagn:
        ret = receive_stab_acc_gyro_magn(stream_data_);
        break;
      case DataType::AccGyroRotMat:
        ret = receive_acc_gyro_rot_mat(stream_data_);
        break;
      case DataType::Quaternion:
        ret = receive_quaternion(stream_data_);
        break;
      default:
        rt_printf("Unknown message of type 0x%02x requested!\n", data_type);
        ret = false;
        break;
      }
    // }
  }

  return ret;
}

bool Imu3DM_GX3_25::stop_reading_loop(void)
{
  mutex_.lock();
  stop_imu_communication_ = true;
  mutex_.unlock();
  thread_.join();
  return true;
}

bool Imu3DM_GX3_25::receive_acc_gyro(bool stream_data)
{
    // rt_printf("Imu3DM_GX3_25::receive_acc_gyro(): [Status] "
    //         "Setting continuous mode (stop streaming data)...\n");
  if(!stream_data)
  {
    // send the command to the IMU
    if (!send_message(acc_gyro_msg_))
    {
      rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Error] "
                "Failed to send the continuous mode (stop streaming data) "
                "message\n");
      return false;
    }
  }

  // Check that the Device received the message.
  if (!receive_message(acc_gyro_msg_, stream_data))
  {
    rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Error] "
              "Failed to receive the continuous mode (stop streaming data) "
              "message reply\n");
    return false;
  }

// #ifdef __XENO__
//   if (debug_timing_)
//   {
//     t2_ = rt_timer_read();
//   }
// #endif

  mutex_.lock();

  float tmp[3];
  memcpy(tmp, &(acc_gyro_msg_.reply_[1]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    acceleration_[i] = tmp[i];
  }
  memcpy(tmp, &(acc_gyro_msg_.reply_[13]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    angular_rate_[i] = tmp[i];
  }
  time_stamp_ = ((acc_gyro_msg_.reply_[25] << 24) |
                 (acc_gyro_msg_.reply_[26] << 16) |
                 (acc_gyro_msg_.reply_[27] << 8)  |
                 (acc_gyro_msg_.reply_[28])) / 62500.0;

  mutex_.unlock();

// #ifdef __XENO__
//   if (debug_timing_)
//   {
//     delta1_ = (t1_ - t3_) / 1000000.0;
//     t3_ = rt_timer_read();
//     delta2_ = (t2_ - t1_) / 1000000.0;
//     delta3_ = (t3_ - t2_) / 1000000.0;
//     rt_fprintf(logfile_, "%f %f %f %f\n", delta1_, delta2_, delta3_, timestamp_);
//   }
// #endif


  // rt_printf("Imu3DM_GX3_25::stop_streaming_data(): [Status] "
  //           "Set continuous mode (stop streaming data) successfully with "
  //           "reply: %s\n",
  //           msg.reply_debug_string().c_str());

  return true;
}

} // namespace imu_3DM_GX3_25
} // namespace imu_core
