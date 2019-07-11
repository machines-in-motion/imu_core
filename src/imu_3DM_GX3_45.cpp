/**
 * Generic interface for the 3DM-GX3-25 IMU for Xenomai and non xenomai machines.
*/

#include "imu-core/imu_3DM_GX3_45.hpp"

using namespace real_time_tools;
namespace imu_core{
namespace imu_3DM_GX3_45{

Imu3DM_GX3_45::Imu3DM_GX3_45(const std::string& port_name,
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
}

Imu3DM_GX3_25::~Imu3DM_GX3_25(void)
{
  stop_reading_loop();
  if (stream_data_)
  {
    stop_streaming_data();
  }
  usb_stream_.close_device();
}

// bool Imu3DM_GX3_25::initialize()
// {
//   // open OS communication
//   bool initialized = open_usb_port();
//   // setup the IMU configuration
//  for(unsigned i=0 ; i<3 ; ++i)
//  {
//     initialized = initialized && stop_streaming_data();
//     initialized = initialized && set_communication_settings();
//     // initialized = initialized && set_sampling_settings();
//     // initialized = initialized && initialize_time_stamp();
//     // initialized = initialized && capture_gyro_bias();
//     if(stream_data_)
//     {
//       initialized = initialized && start_streaming_data();
//     }
//     if(initialized)
//     {
//       break;
//     }
//   }

//   if(initialized)
//   {
//     thread_.create_realtime_thread(Imu3DM_GX3_25::reading_loop_helper, this);
//   }
//   return true;
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
