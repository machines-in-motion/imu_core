/**
 * Generic interface for the 3DM-GX3-25 IMU for Xenomai and non xenomai machines.
*/

#include "imu-core/imu_3DM_GX3_25.hpp"

using namespace real_time_tools;
namespace imu_core{
namespace imu_3DM_GX3_25{

Imu3DM_GX3_25::Imu3DM_GX3_25(const std::string& port_name):
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
}

Imu3DM_GX3_25::~Imu3DM_GX3_25(void)
{
  // stopReadingLoop();
  // if (stream_data_)
  // {
  //   stopStream();
  // }
  // closePort();
  // rt_mutex_destroy(&mutex_);
}

bool Imu3DM_GX3_25::initialize()
{
  // open OS communication
  bool initialized = usb_stream_.open_device(port_name_);
  // setup OS communication
  port_config_.rts_cts_enabled_ = false;
  port_config_.parity_ = false;
  port_config_.stop_bits_ = real_time_tools::PortConfig::StopBits::one;
  port_config_.prepare_size_definition_ = false;
  port_config_.data_bits_ = real_time_tools::PortConfig::cs8;
  port_config_.baude_rate_ = real_time_tools::PortConfig::BR_115200;
  usb_stream_.set_port_config(port_config_);
  usb_stream_.set_poll_mode_timeout(0.005);

  // setup the IMU configuration
  initialized = initialized && set_communication_settings();
  initialized = initialized && setSamplingSettings();
  initialized = initialized && initTimestamp();
  initialized = initialized && captureGyroBias();
  return true;
}

bool Imu3DM_GX3_25::receive_message(ImuMsg& msg, bool stream_mode)
{
  if(!usb_stream_.read_device(msg.reply_, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::receive_message(): [Error] "
              "message badly received\n");
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
    int num_missed = 1;
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
    // we copy the rest of the message in the msg.reply_
    unsigned start_index = msg.reply_.size()-num_missed;
    for(unsigned i=start_index ; i<msg.reply_.size() ; ++i)
    {
      msg.reply_[i] = fragment[i-start_index];
    }
    // in case all the procedure failed we try again.
    ++trial;
  }

  return true;
}

bool Imu3DM_GX3_25::set_communication_settings(void)
{
  rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Status] "
            "Setting communication settings...\n");

  // send the configuration to the IMU
  CommunicationSettingsMsg comm_msg(BaudeRate::BR_921600);
  if (!send_message(comm_msg))
  {
    rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Error] "
              "Failed to send the communication settings\n");
    return false;
  }

  // reset the computer port Baude Rate
  port_config_.baude_rate_ = BaudeRate::BR_921600;
  usb_stream_.set_port_config(port_config_);

  // Check that the Device received the message.
  bool stream_mode = false; // Poll mode
  if (!receive_message(comm_msg, stream_mode))
  {
    rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Error] "
              "Failed to receive the communication settings reply\n");
    return false;
  }

  rt_printf("Imu3DM_GX3_25::set_communication_settings(): [Status] "
            "Set communication settings successfully with reply: %s\n",
            comm_msg.reply_debug_string().c_str());
  return true;
}

bool Imu3DM_GX3_25::set_sampling_settings(void)
{

  print_string("Setting sampling settings...\n");

  // setup sampling config message
  buffer_[0] = CMD_SAMP_SETTINGS;
  buffer_[1] = SAMP_SETTINGS_CONF1;
  buffer_[2] = SAMP_SETTINGS_CONF2;
  buffer_[3] = 1; // change params to new values
  buffer_[4] = 0;
  buffer_[5] = dec_rate_;                // decimation rate
  uint16_t fselect = 0b0000110100010000; // data conditioning function selector
  if (calc_orient_)
  {
    if (calc_quat_)
    {
      fselect = 0b0001010100010011;
    }
    else
    {
      fselect = 0b0000010100010011;
    }
  }
  *(uint16_t *)(&buffer_[6]) = bswap_16(fselect);
  buffer_[8] = 15;
  buffer_[9] = 17;
  buffer_[10] = 0;
  buffer_[11] = 10;
  buffer_[12] = 0;
  buffer_[13] = 10;
  buffer_[14] = 0;
  buffer_[15] = 0;
  buffer_[16] = 0;
  buffer_[17] = 0;
  buffer_[18] = 0;
  buffer_[19] = 0;

  if (!writeToDevice(CMD_SAMP_SETTINGS_LEN))
  {
    print_string("ERROR >> Failed to set sampling settings\n");
    return false;
  }

  if (!readFromDevice(CMD_SAMP_SETTINGS, RPLY_SAMP_SETTINGS_LEN))
  {
    print_string("ERROR >> Failed to set sampling settings\n");
    return false;
  }

  print_string("Set sampling settings successfully with reply: ");
#ifdef __XENO__
  for (int i = 0; i < RPLY_SAMP_SETTINGS_LEN; ++i)
  {
    rt_printf("%02x ", buffer_[i]);
  }
  rt_printf("\n");
#else
  for (int i = 0; i < RPLY_SAMP_SETTINGS_LEN; ++i)
  {
    printf("%02x ", buffer_[i]);
  }
  printf("\n");
#endif

  return true;
}

bool Imu3DM_GX3_25::initTimestamp(void)
{

  buffer_[0] = CMD_TIMER;
  buffer_[1] = TIMER_CONF1;
  buffer_[2] = TIMER_CONF2;
  buffer_[3] = 1;                           // set new timestamp
  *(uint32_t *)(&buffer_[4]) = bswap_32(0); // start at 0

  if (!writeToDevice(CMD_TIMER_LEN))
  {
    print_string("ERROR >> Failed to send initialize timestamp command.\n");
    return false;
  }

  if (!readFromDevice(CMD_TIMER, RPLY_TIMER_LEN))
  {
    print_string("ERROR >> Failed to read initialize timestamp command.\n");
    return false;
  }

  return true;
}

bool Imu3DM_GX3_25::captureGyroBias(void)
{

  uint16_t duration = 3;
#ifdef __XENO__
  rt_printf("Capturing gyro bias for %d seconds... ", duration);
#else
  printf("Capturing gyro bias for %d seconds... ", duration);
#endif

  buffer_[0] = CMD_GYRO_BIAS;
  buffer_[1] = GYRO_BIAS_CONF1;
  buffer_[2] = GYRO_BIAS_CONF2;
  *(uint16_t *)(&buffer_[3]) = bswap_16(duration * 1000);

  if (!writeToDevice(CMD_GYRO_BIAS_LEN))
  {
    print_string("ERROR >> Failed to capture gyro bias.\n");
    return false;
  }

  if (!readFromDevice(CMD_GYRO_BIAS, RPLY_GYRO_BIAS_LEN))
  {
    print_string("ERROR >> Failed to read gyro bias reply.\n");
    return false;
  }

  print_string("done.\n");
  return true;
}

bool Imu3DM_GX3_25::setTimeout(double timeout)
{

#ifdef __XENO__
  // Set read timeout
  rt_config_.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD;
  rt_config_.rx_timeout = (nanosecs_rel_t)(timeout * 1000000000); // rx_timeout in ns
  rt_config_.baud_rate = 921600;
  res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &rt_config_);
  if (res_ != 0)
  {
    rt_printf("ERROR >> Failed to set read timeout.\n");
    return false;
  }
#else
  FD_ZERO(&set_);
  FD_SET(fd_, &set_);
  timeout_.tv_sec = 0;
  timeout_.tv_nsec = (long)(timeout * 1000000000);
#endif

  return true;
}

bool Imu3DM_GX3_25::startStream(void)
{

  // put imu in streaming mode
  buffer_[0] = CMD_CONT_MODE;
  buffer_[1] = CONT_MODE_CONF1;
  buffer_[2] = CONT_MODE_CONF2;
  buffer_[3] = message_type_[0];

  if (!writeToDevice(CMD_CONT_MODE_LEN))
  {
    print_string("ERROR >> Failed to start continuous mode.\n");
    return false;
  }

  if (!readFromDevice(CMD_CONT_MODE, RPLY_CONT_MODE_LEN))
  {
    print_string("ERROR >> Failed to start continuous mode.\n");
    return false;
  }

#ifdef __XENO__
  rt_printf("Continuous mode started, streaming command %02x\n", buffer_[1]);
#else
  printf("Continuous mode started, streaming command %02x\n", buffer_[1]);
#endif

  return true;
}

bool Imu3DM_GX3_25::stopStream(void)
{

  buffer_[0] = CMD_STOP_CONT;
  buffer_[1] = STOP_CONT_CONF1;
  buffer_[2] = STOP_CONT_CONF2;

  if (!writeToDevice(CMD_STOP_CONT_LEN))
  {
    print_string("ERROR >> Failed to stop continuous mode.\n");
    return false;
  }

  return true;
}

bool Imu3DM_GX3_25::closePort(void)
{

  if (is_45_)
  {
    switchMode(false);
  }

#ifdef __XENO__
  res_ = rt_dev_close(fd_);
#else
  res_ = close(fd_);
#endif

  if (res_ != 0)
  {
    print_string("ERROR >> Failed to close port.\n");
    return false;
  }

  return true;
}

bool Imu3DM_GX3_25::readingLoop(void)
{

  // Make this a Xenomai thread and switch to primary mode:
#ifdef __XENO__
  int res_ = rt_task_shadow(NULL, imu_comm_xeno_info_.keyword_, imu_comm_xeno_info_.priority_, T_JOINABLE | T_FPU);
  if (res_ != 0)
  {
    rt_printf("ERROR >> Failed to shadow the calling non-RT task.\n");
    return false;
  }
#endif

#if __XENO__
  if (realtime_)
  {
    rt_task_set_mode(0, T_PRIMARY, NULL);
  }
  else
  {
    rt_task_set_mode(T_PRIMARY, 0, NULL);
  }
#endif

  while (!stop_imu_comm_)
  {
    for (int i = 0; i < num_messages_; ++i)
    {

      switch (message_type_[i])
      {
      case CMD_AC_AN:
        receiveAccelAngrate();
        break;
      case CMD_STAB_AC_AN_MAG:
        receiveStabAccelAngrateMag();
        break;
      case CMD_AC_AN_OR:
        receiveAccelAngrateOrient();
        break;
      case CMD_QUAT:
        receiveQuat();
        break;
      default:
#ifdef __XENO__
        rt_printf("Unknown message of type 0x%02x requested!\n", message_type_[i]);
#else
        printf("Unknown message of type 0x%02x requested!\n", message_type_[i]);
#endif
        break;
      }
    }
  }

  // return THREAD_FUNCTION_RETURN_VALUE;
}

bool Imu3DM_GX3_25::stopReadingLoop(void)
{
  lockData();
  stop_imu_comm_ = true;
  unlockData();
  thread_.join();
  return true;
}

bool Imu3DM_GX3_25::receiveAccelAngrate(void)
{

#ifdef __XENO__
  RTIME t1_, t2_, t3_;
#endif

#ifdef __XENO__
  if (debug_timing_)
  {
    t1_ = rt_timer_read();
  }
#endif

  if (stream_data_)
  {
    if (!readFromDevice(CMD_AC_AN, RPLY_AC_AN_LEN))
    {
      print_string("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_AC_AN;
    if (!writeToDevice(1))
    {
      print_string("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_AC_AN, RPLY_AC_AN_LEN))
    {
      print_string("WARNING >> Failed to read polled message, skipping.\n");
      return false;
    }
  }

#ifdef __XENO__
  if (debug_timing_)
  {
    t2_ = rt_timer_read();
  }
#endif

  lockData();

  float tmp[3];
  memcpy(tmp, &(buffer_[1]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    accel_[i] = tmp[i];
  }

  memcpy(tmp, &(buffer_[13]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    angrate_[i] = tmp[i];
  }

  timestamp_ = ((buffer_[25] << 24) | (buffer_[26] << 16) | (buffer_[27] << 8) | (buffer_[28])) / 62500.0;

  unlockData();

#ifdef __XENO__
  if (debug_timing_)
  {
    delta1_ = (t1_ - t3_) / 1000000.0;
    t3_ = rt_timer_read();
    delta2_ = (t2_ - t1_) / 1000000.0;
    delta3_ = (t3_ - t2_) / 1000000.0;
    rt_fprintf(logfile_, "%f %f %f %f\n", delta1_, delta2_, delta3_, timestamp_);
  }
#endif

  return true;
}

bool Imu3DM_GX3_25::receiveStabAccelAngrateMag(void)
{

  if (stream_data_)
  {
    if (!readFromDevice(CMD_STAB_AC_AN_MAG, RPLY_STAB_AC_AN_MAG_LEN))
    {
      print_string("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_STAB_AC_AN_MAG;
    if (!writeToDevice(1))
    {
      print_string("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_STAB_AC_AN_MAG, RPLY_STAB_AC_AN_MAG_LEN))
    {
      print_string("WARNING >> Failed to read polled message, skipping.\n");
      return false;
    }
  }

  lockData();

  float tmp[3];
  memcpy(tmp, &(buffer_[1]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    stab_accel_[i] = tmp[i];
  }

  memcpy(tmp, &(buffer_[13]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    angrate_[i] = tmp[i];
  }

  memcpy(tmp, &(buffer_[25]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    stab_mag_[i] = tmp[i];
  }

  timestamp_ = ((buffer_[37] << 24) | (buffer_[38] << 16) | (buffer_[39] << 8) | (buffer_[40])) / 62500.0;

  unlockData();

  return true;
}

bool Imu3DM_GX3_25::receiveAccelAngrateOrient(void)
{

  if (stream_data_)
  {
    if (!readFromDevice(CMD_AC_AN_OR, RPLY_AC_AN_OR_LEN))
    {
      print_string("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_AC_AN_OR;
    if (!writeToDevice(1))
    {
      print_string("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_AC_AN_OR, RPLY_AC_AN_OR_LEN))
    {
      print_string("WARNING >> Failed to read polled message, skipping.\n");
      return false;
    }
  }

  lockData();

  float tmp[3];
  memcpy(tmp, &(buffer_[1]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    stab_accel_[i] = tmp[i];
  }

  memcpy(tmp, &(buffer_[13]), 3 * sizeof(float));
  for (int i = 0; i < 3; ++i)
  {
    angrate_[i] = tmp[i];
  }

  float tmp_mat[9];
  memcpy(tmp_mat, &(buffer_[25]), 9 * sizeof(float));
  for (int i = 0; i < 9; ++i)
  {
    orient_mat_[i] = tmp_mat[i];
  }

  timestamp_ = ((buffer_[61] << 24) | (buffer_[62] << 16) | (buffer_[63] << 8) | (buffer_[64])) / 62500.0;

  unlockData();

  return true;
}

bool Imu3DM_GX3_25::receiveQuat(void)
{

  if (stream_data_)
  {
    if (!readFromDevice(CMD_QUAT, RPLY_QUAT_LEN))
    {
      print_string("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_QUAT;
    if (!writeToDevice(1))
    {
      print_string("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_QUAT, RPLY_QUAT_LEN))
    {
      print_string("WARNING >> Failed to read polled message, skipping.\n");
      return false;
    }
  }

  lockData();

  float tmp[4];
  memcpy(tmp, &(buffer_[1]), 4 * sizeof(float));
  for (int i = 0; i < 4; ++i)
  {
    quat_[i] = tmp[i];
  }

  timestamp_ = ((buffer_[17] << 24) | (buffer_[18] << 16) | (buffer_[19] << 8) | (buffer_[20])) / 62500.0;

  unlockData();

  return true;
}

bool Imu3DM_GX3_25::readAccelAngrate(double *accel, double *angrate, double &timestamp)
{

  lockData();

  memcpy(accel, accel_, 3 * sizeof(double));
  memcpy(angrate, angrate_, 3 * sizeof(double));
  timestamp = timestamp_;

  unlockData();

  return true;
}

bool Imu3DM_GX3_25::readStabAccelAngrateMag(double *stab_accel, double *angrate, double *stab_mag, double &timestamp)
{

  lockData();

  memcpy(stab_accel, stab_accel_, 3 * sizeof(double));
  memcpy(angrate, angrate_, 3 * sizeof(double));
  memcpy(stab_mag, stab_mag_, 3 * sizeof(double));
  timestamp = timestamp_;

  unlockData();

  return true;
}

bool Imu3DM_GX3_25::readQuat(double *quat, double &timestamp)
{

  lockData();

  memcpy(quat, quat_, 4 * sizeof(double));
  timestamp = timestamp_;

  unlockData();

  return true;
}

// AUXILIARY FUNCTIONS:

void Imu3DM_GX3_25::print_string(const std::string& string)
{
  rt_printf("%s", string.c_str());
}

void Imu3DM_GX3_25::lockData(void)
{
  rt_mutex_lock(&mutex_);
}

void Imu3DM_GX3_25::unlockData(void)
{
  rt_mutex_unlock(&mutex_);
}

bool Imu3DM_GX3_25::writeToDevice(int len)
{

#ifdef __XENO__
  res_ = rt_dev_write(fd_, buffer_, len);
#else
  res_ = write(fd_, buffer_, len);
#endif

  if (res_ < 0)
  {
    char *error_code = strerror(-res_);
    printf("ERROR >> Writing to device failed with error: %s\n", error_code);
    return false;
  }
  else if (res_ != len)
  {
    printf("ERROR >> Failed writing requested amount of %d bytes\n", len);
    return false;
  }

  return true;
}

bool Imu3DM_GX3_25::readFromDevice(uint8_t command, int len)
{

#ifdef __XENO__
  res_ = rt_dev_read(fd_, buffer_, len);
#else
  if (!stream_data_ && timeout_set_)
  {
    res_ = pselect(fd_ + 1, &set_, NULL, NULL, &timeout_, NULL);
    if (res_ == -1)
    {
      printf("ERROR >> Select error occurred!");
      return false;
    }
    else if (res_ == 0)
    {
      printf("WARNING >> Timeout reached when reading command 0x%02x.\n", command);
      return false;
    }
    else
    {
      res_ = read(fd_, buffer_, len);
    }
  }
  else
  {
    res_ = read(fd_, buffer_, len); // when streaming, no read timeout
  }
#endif

  if (res_ < 0)
  {
    char *error_code = strerror(-res_);
    if (strcmp(error_code, "Connection timed out") == 0)
    {
      rt_printf("WARNING >> Timeout reached when reading command 0x%02x.\n", command);
    }
    else
    {
      rt_printf("ERROR >> Reading from device failed with error: %s\n", error_code);
    }
    return false;
  }
  else if (res_ != len)
  {
    rt_printf("ERROR >> Reading from device failed. Requested %d bytes and received %ld bytes: ", len, res_);
    for (int i = 0; i < res_; ++i)
    {
      rt_printf("%02x ", buffer_[i]);
    }
    rt_printf("\n");
    return false;
  }

  return true;
}



// NOT CURRENTLY USED:


/////////////// C interface //////////
Imu3DM_GX3_25 myIMU("ttyACM0", true, false, false);
extern "C"
{
  int initialize_imu_interface(uint8_t *message_type, int num_messages);
  int read_accel_angrate(double *accel, double *angrate, double timestamp);
} // extern "C"
int initialize_imu_interface(uint8_t *message_type, int num_messages)
{
  if (myIMU.initialize(message_type, num_messages))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int read_accel_angrate(double *accel, double *angrate, double timestamp)
{
  if (myIMU.readAccelAngrate(accel, angrate, timestamp))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

} // namespace imu_3DM_GX3_25
} // namespace imu_core















































  // // Message incorrect
  // if (!is_checksum_correct(msg))
  // {
  //   rt_printf("UsbStream::read_device: "
  //             "Received message %s with bad checksum from command %s.",
  //             msg.reply_debug_string().c_str(),
  //             msg.command_debug_string().c_str());
  //   if (stream_mode_on_)
  //   {
  //     rt_printf("UsbStream::read_device: WARNING, Attempting to re-align with "
  //               "stream...\n");
  //     return read_misaligned_msg_from_device(msg);
  //   }
  //   return false;
  // }
  // else if (is_header_found(msg))
  // {
  //   rt_printf("ERROR >> Received unexpected message from device.\n");
  //   return false;
  // }


// bool UsbStream::is_checksum_correct(uint8_t *rep, int rep_len)
// {
//   uint16_t checksum = 0;
//   for (int i = 0; i < rep_len - 2; i++)
//   {
//     checksum += ((uint8_t *)rep)[i];
//   }

//   return checksum == bswap_16(*(uint16_t *)((uint8_t *)rep + rep_len - 2));
// }

// bool UsbStream::read_misaligned_msg_from_device(
//   const std::vector<uint8_t>& command,
//   std::vector<uint8_t>& reply)
// {

//   // When we read corrupt data, try to keep reading until we catch up with clean data:
//   int trial = 0;
//   while (cmd_buffer_[0] != cmd || !is_checksum_correct(cmd_buffer_.data(), len))
//   {
//     if (trial >= max_realign_trials_)
//     {
//       rt_printf("ERROR >> Realigning failed!\n");
//       return false;
//     }

//     // Print the corrupt message:
//     rt_printf("WARNING >> Read invalid message: ");
//     for (int i = 0; i < len; ++i)
//     {
//       rt_printf("%02x ", cmd_buffer_[i]);
//     }
//     rt_printf("\n");

//     // Search for the header:
//     int num_missed = 1;
//     for (; num_missed < len; ++num_missed)
//     {
//       if (cmd == cmd_buffer_[num_missed])
//       {
//         break;
//       }
//     }

//     if (num_missed >= len)
//     {
//       rt_printf("ERROR >> Realigning failed!\n");
//       return false;
//     }

//     // We MIGHT have found the header!
//     uint8_t fragment[len];
// #if defined(XENOMAI)
//     return_value_ = rt_dev_read(file_id_, fragment, num_missed);
// #elif defined(RT_PREEMPT) || defined(NON_REAL_TIME)
//     return_value_ = read(file_id_, fragment, num_missed);
// #endif
//     if (return_value_ != num_missed)
//     {
//       int errsv = errno;
//       rt_printf("ERROR >> Failed to read fragment from port %s with error\n"
//               "\t%s\n", file_name_.c_str(), strerror(errsv));
//       return false;
//     }

//     uint8_t tmp_buf[len];
//     memcpy(tmp_buf, &cmd_buffer_[num_missed], (len - num_missed) * sizeof(uint8_t));
//     memcpy(&tmp_buf[len - num_missed], fragment, num_missed * sizeof(uint8_t));
//     memcpy(cmd_buffer_.data(), tmp_buf, len * sizeof(uint8_t));

//     ++trial;
//   }

//   return true;
// }

// bool is_header_found(msg)
// {
//   bool header_found = true;
//   for (unsigned i=0; i<msg.command.size())
//   {
//     header_found = header_found && (msg.command_[i] != msg.reply_[i])
//   }
// }




































// bool Imu3DM_GX3_25::switchMode(bool to_25)
// {

//   int cmd_len = 8;
//   int rep_len = 10;
//   // set IMU GX3-45 to idle
//   buffer_[0] = 0x75; // sync1
//   buffer_[1] = 0x65; // sync2
//   buffer_[2] = 0x01; // descriptor set (system command)
//   buffer_[3] = 0x02; // payload length
//   buffer_[4] = 0x02; // field length
//   buffer_[5] = 0x02; // field descriptor (communication mode)
//   buffer_[6] = 0xE1; // checksum MSB
//   buffer_[7] = 0xC7; // checksum LSB
// #ifdef __XENO__
//   rt_dev_write(fd_, buffer_, cmd_len);
//   rt_dev_read(fd_, buffer_, rep_len);
// #else
//   write(fd_, buffer_, cmd_len);
//   read(fd_, buffer_, rep_len);
// #endif

//   // Compute Fletcher Checksum:
//   uint8_t checksum_byte1 = 0;
//   uint8_t checksum_byte2 = 0;
//   uint16_t checksum = 0;
//   for (int i = 0; i < rep_len - 2; ++i)
//   {
//     checksum_byte1 += buffer_[i];
//     checksum_byte2 += checksum_byte1;
//   }
//   checksum = (checksum_byte1 << 8) | checksum_byte2;
//   if (checksum != ((buffer_[rep_len - 2] << 8) | buffer_[rep_len - 1]))
//   {
//     print_string("Invalid Fletcher Checksum! Message: ");
// #ifdef __XENO__
//     for (int i = 0; i < rep_len; ++i)
//     {
//       rt_printf("%02x ", buffer_[i]);
//     }
// #else
//     for (int i = 0; i < rep_len; ++i)
//     {
//       printf("%02x ", buffer_[i]);
//     }
// #endif
//     print_string("\n");
//     return false;
//   }

// #ifdef __XENO__
//   for (int i = 0; i < rep_len; ++i)
//   {
//     rt_printf("%02x ", buffer_[i]);
//   }
// #else
//   for (int i = 0; i < rep_len; ++i)
//   {
//     printf("%02x ", buffer_[i]);
//   }
// #endif
//   print_string("\n");

//   cmd_len = 10;
//   rep_len = 10;

//   // Communication Mode (0x7F, 0x10)
//   buffer_[0] = 0x75; // sync1
//   buffer_[1] = 0x65; // sync2
//   buffer_[2] = 0x7F; // descriptor set (system command)
//   buffer_[3] = 0x04; // payload length
//   buffer_[4] = 0x04; // field length
//   buffer_[5] = 0x10; // field descriptor (communication mode)
//   buffer_[6] = 0x01; // use new settings
//   if (to_25)
//   {                    // switch to 3DMGX3-25 mode:
//     buffer_[7] = 0x02; // AHRS Direct (3dmgx3-25 single byte protocol)
//     buffer_[8] = 0x74; // checksum MSB
//     buffer_[9] = 0xBD; // checksum LSB
//   }
//   else
//   {                    // switch back to 3DMGX3-45 mode:
//     buffer_[7] = 0x01; // NAV (3dmgx3-45 MIP protocol)
//     buffer_[8] = 0x73; // checksum MSB
//     buffer_[9] = 0xBC; // checksum LSB
//   }

// #ifdef __XENO__
//   rt_dev_write(fd_, buffer_, cmd_len);
//   rt_dev_read(fd_, buffer_, rep_len);
// #else
//   write(fd_, buffer_, cmd_len);
//   read(fd_, buffer_, rep_len);
// #endif

//   // Compute Fletcher Checksum:
//   checksum_byte1 = 0;
//   checksum_byte2 = 0;
//   checksum = 0;
//   for (int i = 0; i < rep_len - 2; ++i)
//   {
//     checksum_byte1 += buffer_[i];
//     checksum_byte2 += checksum_byte1;
//   }
//   checksum = (checksum_byte1 << 8) | checksum_byte2;
//   if (checksum != ((buffer_[rep_len - 2] << 8) | buffer_[rep_len - 1]))
//   {
//     print_string("Invalid Fletcher Checksum! Message: ");
// #ifdef __XENO__
//     for (int i = 0; i < rep_len; ++i)
//     {
//       rt_printf("%02x ", buffer_[i]);
//     }
// #else
//     for (int i = 0; i < rep_len; ++i)
//     {
//       printf("%02x ", buffer_[i]);
//     }
// #endif
//     print_string("\n");
//     return false;
//   }

// #ifdef __XENO__
//   for (int i = 0; i < rep_len; ++i)
//   {
//     rt_printf("%02x ", buffer_[i]);
//   }
// #else
//   for (int i = 0; i < rep_len; ++i)
//   {
//     printf("%02x ", buffer_[i]);
//   }
// #endif
//   print_string("\n");

//   print_string("Successfully switched to AHRS direct mode.\n");

// #if __XENO__
//   rt_task_sleep(1000000000); // 1s
// #else
//   usleep(1000);
// #endif

//   return true;
// }