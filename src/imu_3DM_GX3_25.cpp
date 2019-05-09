/*
/"
 * Generic interface for the 3DM-GX3-25 IMU for Xenomai and non xenomai machines.
*/

#include "imu-core/imu_3DM_GX3_25.hpp"

using namespace real_time_tools;

Imu3DM_GX3_25::Imu3DM_GX3_25(const char *portname, bool stream_data, bool real_time, bool is_45)
{

  rt_mutex_init(&mutex_);
  imu_comm_thread_.reset();
  stop_imu_comm_ = true;
  stop_imu_comm_ = true;
  timeout_set_ = false;
  debug_timing_ = false; // set to true to record timestamps
  port_ = portname;
  stream_data_ = stream_data;
  realtime_ = real_time;
  is_45_ = is_45;

  // Set default streaming thread parameters:
  imu_comm_xeno_info_.keyword_ = "imu_comm";
  imu_comm_xeno_info_.priority_ = 25;
  imu_comm_xeno_info_.cpu_id_ = 4;
}

Imu3DM_GX3_25::~Imu3DM_GX3_25(void)
{
  stopReadingLoop();
  if (stream_data_)
  {
    stopStream();
  }
  closePort();
  rt_mutex_destroy(&mutex_);
}

bool Imu3DM_GX3_25::initialize(uint8_t *message_type, int num_messages)
{

  num_messages_ = num_messages;
  message_type_ = message_type;

  if (num_messages_ > 1 && stream_data_)
  {
    printString("WARNING: Cannot stream more than one message type.  Switching to polling mode.\n");
    stream_data_ = false;
  }

  calc_orient_ = false;
  calc_quat_ = false;
  dec_rate_ = 1; // 1000 hz
  for (int i = 0; i < num_messages_; ++i)
  {
    if (message_type_[i] == CMD_AC_AN_OR)
    {
      calc_orient_ = true;
      dec_rate_ = 2; // 500 hz
      printString("WARNING: Orientation matrix calculation enabled, max data rate of 500hz set.\n");
    }
    if (message_type_[i] == CMD_QUAT)
    {
      calc_orient_ = true;
      calc_quat_ = true;
      dec_rate_ = 2; // 500 hz
      printString("WARNING: Quaternion calculation enabled, max data rate of 500hz set.\n");
    }
  }

  bool initialized = openPort();
  initialized = initialized && setCommunicationSettings();
  initialized = initialized && setSamplingSettings();
  initialized = initialized && initTimestamp();
  initialized = initialized && captureGyroBias();
  if (stream_data_)
  {
    initialized = initialized && startStream();
  }
  else
  {
    setTimeout(0.005); // timeout in seconds
    timeout_set_ = true;
  }

  if (!initialized)
  {
    printString("ERROR >> IMU initialization failed.\n");
    return false;
  }

  // // start IMU reading thread
  stop_imu_comm_ = false;
  imu_comm_thread_.reset(new boost::thread(boost::bind(&Imu3DM_GX3_25::readingLoop, this)));

  return true;
}

bool Imu3DM_GX3_25::setStreamThreadParams(char *keyword, int priority, int stacksize, int cpu_id, int delay_ns)
{

  imu_comm_xeno_info_.keyword_ = keyword;
  imu_comm_xeno_info_.priority_ = priority;
  imu_comm_xeno_info_.stacksize_ = stacksize;
  imu_comm_xeno_info_.cpu_id_ = cpu_id;
  imu_comm_xeno_info_.delay_ns_ = delay_ns;

  return true;
}

bool Imu3DM_GX3_25::openPort(void)
{

  if (debug_timing_)
  {
    char str[100];
    snprintf(str, sizeof(str), "%s%s", "imu_timing_log_", port_);
    logfile_ = fopen(str, "w");
#ifdef __XENO__
    rt_print_auto_init(1); // for real-time safe printing
#endif
  }

#ifdef __XENO__
  fd_ = rt_dev_open(port_, O_RDWR);
  if (fd_ < 0)
  {
    rt_printf("ERROR >> Failed to open real-time USB port %s.  Are you sure you've loaded the correct drivers?\n", port_);
    return false;
  }
#else
  char str[100];
  snprintf(str, sizeof(str), "%s%s", "/dev/", port_);
  fd_ = open(str, O_RDWR | O_SYNC); // blocking mode by default, unless O_NONBLOCK is passed
  if (fd_ < 0)
  {
    printf("ERROR >> Failed to open USB port /dev/%s.\n", port_);
    return false;
  }
#endif

#ifdef __XENO__
  // Switch to non-blocking mode and flush the buffer:
  rt_config_.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD;
  rt_config_.rx_timeout = RTSER_TIMEOUT_NONE; // set non-blocking
  rt_config_.baud_rate = 115200;
  res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &rt_config_);
  if (res_ != 0)
  {
    rt_printf("ERROR >> Failed to configure port.\n");
    return false;
  }

  int i = 100;
  while (--i > 0)
  {
    rt_task_sleep(1000000);
    while (rt_dev_read(fd_, buffer_, 100) > 0) // flush buffer and make sure it's cleared for 100*(1000000ns) or 100ms
      i = 100;
  }

  // Switch back to blocking mode:
  rt_config_.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD;
  rt_config_.rx_timeout = RTSER_TIMEOUT_INFINITE; // set blocking
  rt_config_.baud_rate = 115200;
  res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &rt_config_);
  if (res_ != 0)
  {
    rt_printf("ERROR >> Failed to configure port.\n");
    return false;
  }

#else
  // Change port settings
  tcgetattr(fd_, &config_);

  // set port control modes:
  config_.c_cflag = CLOCAL | CREAD;

  // set to 8N1 (eight data bits, no parity bit, one stop bit):
  config_.c_cflag &= ~PARENB;
  config_.c_cflag &= ~CSTOPB;
  config_.c_cflag &= ~CSIZE;
  config_.c_cflag |= CS8;

  // set to baudrate 115200
  cfsetispeed(&config_, B115200);
  cfsetospeed(&config_, B115200);

  // set port properties after flushing buffer
  if (tcsetattr(fd_, TCSAFLUSH, &config_) < 0)
  {
    printf("ERROR >> Failed to configure port.\n");
    return false;
  }
#endif

  printString("IMU port has been opened in ");
#ifdef __XENO__
  printString("real-time ");
#else
  printString("non real-time ");
#endif
  if (stream_data_)
  {
    printString("streaming mode.\n");
  }
  else
  {
    printString("polling mode.\n");
  }
  printString("Message types: ");
  for (int i = 0; i < num_messages_; ++i)
  {
    message_type_[i] = message_type_[i];
#ifdef __XENO__
    rt_printf("0x%02x ", message_type_[i]);
#else
    printf("0x%02x ", message_type_[i]);
#endif
  }
  printString("\n");

  return true;
}

bool Imu3DM_GX3_25::switchMode(bool to_25)
{

  int cmd_len = 8;
  int rep_len = 10;

  buffer_[0] = 0x75; // sync1
  buffer_[1] = 0x65; // sync2
  buffer_[2] = 0x01; // descriptor set (system command)
  buffer_[3] = 0x02; // payload length
  buffer_[4] = 0x02; // field length
  buffer_[5] = 0x02; // field descriptor (communication mode)
  buffer_[6] = 0xE1; // checksum MSB
  buffer_[7] = 0xC7; // checksum LSB
#ifdef __XENO__
  rt_dev_write(fd_, buffer_, cmd_len);
  rt_dev_read(fd_, buffer_, rep_len);
#else
  write(fd_, buffer_, cmd_len);
  read(fd_, buffer_, rep_len);
#endif

  // Compute Fletcher Checksum:
  uint8_t checksum_byte1 = 0;
  uint8_t checksum_byte2 = 0;
  uint16_t checksum = 0;
  for (int i = 0; i < rep_len - 2; ++i)
  {
    checksum_byte1 += buffer_[i];
    checksum_byte2 += checksum_byte1;
  }
  checksum = (checksum_byte1 << 8) | checksum_byte2;
  if (checksum != ((buffer_[rep_len - 2] << 8) | buffer_[rep_len - 1]))
  {
    printString("Invalid Fletcher Checksum! Message: ");
#ifdef __XENO__
    for (int i = 0; i < rep_len; ++i)
    {
      rt_printf("%02x ", buffer_[i]);
    }
#else
    for (int i = 0; i < rep_len; ++i)
    {
      printf("%02x ", buffer_[i]);
    }
#endif
    printString("\n");
    return false;
  }

#ifdef __XENO__
  for (int i = 0; i < rep_len; ++i)
  {
    rt_printf("%02x ", buffer_[i]);
  }
#else
  for (int i = 0; i < rep_len; ++i)
  {
    printf("%02x ", buffer_[i]);
  }
#endif
  printString("\n");

  cmd_len = 10;
  rep_len = 10;

  buffer_[0] = 0x75; // sync1
  buffer_[1] = 0x65; // sync2
  buffer_[2] = 0x7F; // descriptor set (system command)
  buffer_[3] = 0x04; // payload length
  buffer_[4] = 0x04; // field length
  buffer_[5] = 0x10; // field descriptor (communication mode)
  buffer_[6] = 0x01; // use
  if (to_25)
  {                    // switch to 3DMGX3-25 mode:
    buffer_[7] = 0x02; // AHRS Direct (3dmgx3-25 single byte protocol)
    buffer_[8] = 0x74; // checksum MSB
    buffer_[9] = 0xBD; // checksum LSB
  }
  else
  {                    // switch back to 3DMGX3-45 mode:
    buffer_[7] = 0x01; // NAV (3dmgx3-45 MIP protocol)
    buffer_[8] = 0x73; // checksum MSB
    buffer_[9] = 0xBC; // checksum LSB
  }

#ifdef __XENO__
  rt_dev_write(fd_, buffer_, cmd_len);
  rt_dev_read(fd_, buffer_, rep_len);
#else
  write(fd_, buffer_, cmd_len);
  read(fd_, buffer_, rep_len);
#endif

  // Compute Fletcher Checksum:
  checksum_byte1 = 0;
  checksum_byte2 = 0;
  checksum = 0;
  for (int i = 0; i < rep_len - 2; ++i)
  {
    checksum_byte1 += buffer_[i];
    checksum_byte2 += checksum_byte1;
  }
  checksum = (checksum_byte1 << 8) | checksum_byte2;
  if (checksum != ((buffer_[rep_len - 2] << 8) | buffer_[rep_len - 1]))
  {
    printString("Invalid Fletcher Checksum! Message: ");
#ifdef __XENO__
    for (int i = 0; i < rep_len; ++i)
    {
      rt_printf("%02x ", buffer_[i]);
    }
#else
    for (int i = 0; i < rep_len; ++i)
    {
      printf("%02x ", buffer_[i]);
    }
#endif
    printString("\n");
    return false;
  }

#ifdef __XENO__
  for (int i = 0; i < rep_len; ++i)
  {
    rt_printf("%02x ", buffer_[i]);
  }
#else
  for (int i = 0; i < rep_len; ++i)
  {
    printf("%02x ", buffer_[i]);
  }
#endif
  printString("\n");

  printString("Successfully switched to AHRS direct mode.\n");

#if __XENO__
  rt_task_sleep(1000000000); // 1s
#else
  usleep(1000);
#endif

  return true;
}

bool Imu3DM_GX3_25::setCommunicationSettings(void)
{

  if (is_45_)
  {
    switchMode(true);
  }

  printString("Setting communication settings...\n");

  //increase baudrate on imu
  buffer_[0] = CMD_COMM_SETTINGS;
  buffer_[1] = COMM_SETTINGS_CONF1;
  buffer_[2] = COMM_SETTINGS_CONF2;
  buffer_[3] = (uint8_t)1;
  buffer_[4] = (uint8_t)1;
  uint32_t baudrate = 921600;
  *(uint32_t *)(&buffer_[5]) = bswap_32(baudrate);
  buffer_[9] = (uint8_t)2;
  buffer_[10] = (uint8_t)0;

  if (!writeToDevice(CMD_COMM_SETTINGS_LEN))
  {
    printString("ERROR >> Failed to set communication settings\n");
    return false;
  }

#ifdef __XENO__
  //increase baudrate on port
  rt_config_.config_mask = RTSER_SET_TIMEOUT_RX | RTSER_SET_BAUD;
  rt_config_.rx_timeout = RTSER_TIMEOUT_INFINITE; // set blocking
  rt_config_.baud_rate = 921600;
  res_ = rt_dev_ioctl(fd_, RTSER_RTIOC_SET_CONFIG, &rt_config_);
  if (res_ != 0)
  {
    rt_printf("ERROR >> Failed to configure port after changing IMU baudrate.\n");
    return false;
  }
#else
  // Change port settings
  tcgetattr(fd_, &config_);
  // set port control modes:
  config_.c_cflag = CLOCAL | CREAD;
  // set to 8N1 (eight data bits, no parity bit, one stop bit):
  config_.c_cflag &= ~PARENB;
  config_.c_cflag &= ~CSTOPB;
  config_.c_cflag &= ~CSIZE;
  config_.c_cflag |= CS8;
  // set to baudrate 921600
  cfsetispeed(&config_, B921600);
  cfsetospeed(&config_, B921600);
  // set port properties after flushing buffer
  if (tcsetattr(fd_, TCSAFLUSH, &config_) < 0)
  {
    printf("ERROR >> Failed to configure port after changing IMU baudrate.\n");
    return false;
  }
#endif

  if (!readFromDevice(CMD_COMM_SETTINGS, RPLY_COMM_SETTINGS_LEN))
  {
    printString("ERROR >> Failed to set communication settings\n");
    return false;
  }

  printString("Set communication settings successfully with reply: ");
#ifdef __XENO__
  for (int i = 0; i < RPLY_COMM_SETTINGS_LEN; ++i)
  {
    rt_printf("%02x ", buffer_[i]);
  }
  rt_printf("\n");
#else
  for (int i = 0; i < RPLY_COMM_SETTINGS_LEN; ++i)
  {
    printf("%02x ", buffer_[i]);
  }
  printf("\n");
#endif

  return true;
}

bool Imu3DM_GX3_25::setSamplingSettings(void)
{

  printString("Setting sampling settings...\n");

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
    printString("ERROR >> Failed to set sampling settings\n");
    return false;
  }

  if (!readFromDevice(CMD_SAMP_SETTINGS, RPLY_SAMP_SETTINGS_LEN))
  {
    printString("ERROR >> Failed to set sampling settings\n");
    return false;
  }

  printString("Set sampling settings successfully with reply: ");
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
    printString("ERROR >> Failed to send initialize timestamp command.\n");
    return false;
  }

  if (!readFromDevice(CMD_TIMER, RPLY_TIMER_LEN))
  {
    printString("ERROR >> Failed to read initialize timestamp command.\n");
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
    printString("ERROR >> Failed to capture gyro bias.\n");
    return false;
  }

  if (!readFromDevice(CMD_GYRO_BIAS, RPLY_GYRO_BIAS_LEN))
  {
    printString("ERROR >> Failed to read gyro bias reply.\n");
    return false;
  }

  printString("done.\n");
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
    printString("ERROR >> Failed to start continuous mode.\n");
    return false;
  }

  if (!readFromDevice(CMD_CONT_MODE, RPLY_CONT_MODE_LEN))
  {
    printString("ERROR >> Failed to start continuous mode.\n");
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
    printString("ERROR >> Failed to stop continuous mode.\n");
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
    printString("ERROR >> Failed to close port.\n");
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

  return true;
}

bool Imu3DM_GX3_25::stopReadingLoop(void)
{

  if (imu_comm_thread_ != NULL)
  {

    lockData();
    stop_imu_comm_ = true;
    unlockData();

    imu_comm_thread_->join();
  }

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
      printString("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_AC_AN;
    if (!writeToDevice(1))
    {
      printString("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_AC_AN, RPLY_AC_AN_LEN))
    {
      printString("WARNING >> Failed to read polled message, skipping.\n");
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
      printString("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_STAB_AC_AN_MAG;
    if (!writeToDevice(1))
    {
      printString("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_STAB_AC_AN_MAG, RPLY_STAB_AC_AN_MAG_LEN))
    {
      printString("WARNING >> Failed to read polled message, skipping.\n");
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
      printString("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_AC_AN_OR;
    if (!writeToDevice(1))
    {
      printString("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_AC_AN_OR, RPLY_AC_AN_OR_LEN))
    {
      printString("WARNING >> Failed to read polled message, skipping.\n");
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
      printString("ERROR >> Failed to read streamed message.\n");
      return false;
    }
  }
  else
  {
    buffer_[0] = CMD_QUAT;
    if (!writeToDevice(1))
    {
      printString("ERROR >> Failed to send poll for data.\n");
      return false;
    }
    if (!readFromDevice(CMD_QUAT, RPLY_QUAT_LEN))
    {
      printString("WARNING >> Failed to read polled message, skipping.\n");
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

void Imu3DM_GX3_25::printString(const char *string)
{
  rt_printf(string);
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
#ifdef __XENO__
      rt_printf("WARNING >> Timeout reached when reading command 0x%02x.\n", command);
#else
      printf("WARNING >> Timeout reached when reading command 0x%02x.\n", command);
#endif
    }
    else
    {
#ifdef __XENO__
      rt_printf("ERROR >> Reading from device failed with error: %s\n", error_code);
#else
      printf("ERROR >> Reading from device failed with error: %s\n", error_code);
#endif
    }
    return false;
  }
  else if (res_ != len)
  {
#ifdef __XENO__
    rt_printf("ERROR >> Reading from device failed. Requested %d bytes and received %d bytes: ", len, res_);
    for (int i = 0; i < res_; ++i)
    {
      rt_printf("%02x ", buffer_[i]);
    }
    rt_printf("\n");
#else
    printf("ERROR >> Reading from device failed. Requested %d bytes and received %d bytes: ", len, res_);
    for (int i = 0; i < res_; ++i)
    {
      printf("%02x ", buffer_[i]);
    }
    printf("\n");
#endif
    return false;
  }

  if (!isChecksumCorrect(buffer_, len))
  {
    printString("ERROR >> Received message with bad checksum: ");
#ifdef __XENO__
    for (int i = 0; i < len; ++i)
    {
      rt_printf("%02x ", buffer_[i]);
    }
    rt_printf("\n");
#else
    for (int i = 0; i < len; ++i)
    {
      printf("%02x ", buffer_[i]);
    }
    printf("\n");
#endif
    if (stream_data_)
    {
      printString("WARNING >> Attempting to re-align with stream...\n");
      return readMisalignedMsgFromDevice(command, len);
    }
    return false;
  }
  else if (buffer_[0] != command)
  {
    printString("ERROR >> Received unexpected message from device.\n");
    return false;
  }

  return true;
}

uint16_t Imu3DM_GX3_25::bswap_16(uint16_t x)
{
  return (x >> 8) | (x << 8);
}

uint32_t Imu3DM_GX3_25::bswap_32(uint32_t x)
{
  return (bswap_16(x & 0xffff) << 16) | (bswap_16(x >> 16));
}

bool Imu3DM_GX3_25::isChecksumCorrect(uint8_t *rep, int rep_len)
{

  uint16_t checksum = 0;
  for (int i = 0; i < rep_len - 2; i++)
  {
    checksum += ((uint8_t *)rep)[i];
  }

  return checksum == bswap_16(*(uint16_t *)((uint8_t *)rep + rep_len - 2));
}

// NOT CURRENTLY USED:
bool Imu3DM_GX3_25::readMisalignedMsgFromDevice(uint8_t cmd, int len)
{

  // When we read corrupt data, try to keep reading until we catch up with clean data:
  int trial = 0;
  while (buffer_[0] != cmd || !isChecksumCorrect(buffer_, len))
  {
    if (trial >= max_realign_trials_)
    {
      printString("ERROR >> Realigning failed!\n");
      return false;
    }

    // Print the corrupt message:
    printString("WARNING >> Read invalid message: ");
    for (int i = 0; i < len; ++i)
    {
#ifdef __XENO__
      rt_printf("%02x ", buffer_[i]);
#else
      printf("%02x ", buffer_[i]);
#endif
    }
    printString("\n");

    // Search for the header:
    int num_missed = 1;
    for (; num_missed < len; ++num_missed)
    {
      if (cmd == buffer_[num_missed])
      {
        break;
      }
    }

    if (num_missed >= len)
    {
      printString("ERROR >> Realigning failed!\n");
      return false;
    }

    // We MIGHT have found the header!
    uint8_t fragment[len];
#ifdef __XENO__
    res_ = rt_dev_read(fd_, fragment, num_missed);
#else
    res_ = read(fd_, fragment, num_missed);
#endif

    if (res_ != num_missed)
    {
      printString("ERROR >> Could not read fragment.\n");
      return false;
    }

    uint8_t tmp_buf[len];
    memcpy(tmp_buf, &buffer_[num_missed], (len - num_missed) * sizeof(uint8_t));
    memcpy(&tmp_buf[len - num_missed], fragment, num_missed * sizeof(uint8_t));
    memcpy(buffer_, tmp_buf, len * sizeof(uint8_t));

    ++trial;
  }

  return true;
}

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
