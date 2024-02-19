/**
 * @file imu_3DM_GX5_25_msg.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-09-13
 * 
 * Messages specific to the 3DM-GX5-25 IMU protocole
 */

#ifndef IMU_3DM_GX3_25_MSG_HPP
#define IMU_3DM_GX3_25_MSG_HPP

#include "imu-core/imu_interface.hpp"

namespace imu_core
{
namespace imu_3DM_GX5_25
{

/**
 * @brief Simple class that add a field where the expected reply is registered.
 */
class GX5ImuMsg: public ImuMsg
{
public:
  /**
   * @brief Construct a new GX5ImuMsg object
   */
  GX5ImuMsg(): ImuMsg(), sync1_(0x75), sync2_(0x65)
  {}

  /**
   * @brief Comput the checksum fo a buffer
   * 
   * @param cheksum_msb_ 
   * @param cheksum_lsb_ 
   */
  static bool compute_checksum(std::vector<uint8_t> buffer,
                               uint8_t& cheksum_msb_,
                               uint8_t& cheksum_lsb_)
  {
    cheksum_msb_ = 0;
    cheksum_lsb_ = 0;
    if (buffer.size() < 2)
    {
      return false;
    }
    for (unsigned int i=0 ; i<buffer.size()-2 ; ++i)
    {
      cheksum_msb_ += buffer[i];
      cheksum_lsb_ += cheksum_msb_;
    }
    return true;
  }

  /**
   * @brief This is the expected reply from the IMU.
   */
  std::vector<uint8_t> expected_reply_;

protected:

  void construct_command()
  {
    command_.resize(4 + cmd_field_data_.size() + 2);
    command_[0] = sync1_;
    command_[1] = sync2_;
    command_[2] = descr_set_;
    command_[3] = static_cast<uint8_t>(cmd_field_data_.size());
    for(unsigned i = 0 ; i < cmd_field_data_.size() ; ++i)
    {
      command_[4 + i] = cmd_field_data_[i];
    }
    compute_checksum(command_, cheksum_msb_, cheksum_lsb_);
    command_[4 + cmd_field_data_.size()] = cheksum_msb_;
    command_[4 + cmd_field_data_.size() + 1] = cheksum_lsb_;
  }

  void construct_expected_reply()
  {
    expected_reply_.resize(4 + reply_field_data_.size() + 2);
    expected_reply_[0] = sync1_;
    expected_reply_[1] = sync2_;
    expected_reply_[2] = descr_set_;
    expected_reply_[3] = static_cast<uint8_t>(reply_field_data_.size());
    for(unsigned i = 0 ; i < reply_field_data_.size() ; ++i)
    {
      expected_reply_[4 + i] = reply_field_data_[i];
    }
    compute_checksum(expected_reply_, cheksum_msb_, cheksum_lsb_);
    expected_reply_[4 + reply_field_data_.size()] = cheksum_msb_;
    expected_reply_[4 + reply_field_data_.size() + 1] = cheksum_lsb_;
  }

  const uint8_t sync1_; /*!< Synchronisation byte 1 */
  const uint8_t sync2_; /*!< Synchronisation byte 2 */
  /**
   * @brief The “DescriptorSet” byte in the header specifies which command or
   * data set is contained in fields of the packet.
   */
  uint8_t descr_set_ ;
  /**
   * @brief The field data can be any thing but is always rigidly defined.
   * The definition of a descriptor is fundamentally described in a “.h”
   * file that corresponds to the descriptor set that the descriptor belongs to.
   */
  std::vector<uint8_t> cmd_field_data_;
  std::vector<uint8_t> reply_field_data_; /*!< See cmd_field_data_ */
  uint8_t cheksum_msb_; /*!< Checksum msb */
  uint8_t cheksum_lsb_; /*!< Checksum lsb */
};

/**
 * @brief This message allows us to put the Device in Idle Mode.
 * Command: 75 65 01 02 02 02 E1 C7
 * reply: 75 65 01 04 04 F1 [Commandecho:02 or Errorcode:00] D6 6C
 */
class IdleModeMsg: public GX5ImuMsg
{
public:
  /**
   * @brief Construct a new IdleModeMsg object
   */
  IdleModeMsg(): GX5ImuMsg()
  {
    descr_set_ = 0x01;
    cmd_field_data_.resize(2);
    cmd_field_data_[0] = static_cast<uint8_t>(cmd_field_data_.size());
    cmd_field_data_[1] = 0x02;
    construct_command();
    std::vector<uint8_t> expected_command = 
      {0x75, 0x65, 0x01, 0x02, 0x02, 0x02, 0xE1, 0xC7};
    assert(command_ == expected_command && "Idle command wrong");

    reply_field_data_.resize(4);
    reply_field_data_[0] = static_cast<uint8_t>(reply_field_data_.size());
    reply_field_data_[1] = 0xF1;
    reply_field_data_[2] = 0x02;
    reply_field_data_[3] = 0x00;
    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x01, 0x04, 0x04, 0xF1, 0x02, 0x00, 0xD6, 0x6C};
    assert(expected_reply_ == expected_reply && "Idle expected reply wrong");

    reply_.resize(expected_reply_.size());
  }
};

/**
 * @brief IMU data: acc+gyr at 1000Hz
 */
class AccGyr1kHzMsg: public GX5ImuMsg
{
public:

  /**
   * @brief Construct a new AccGyr1kHzMsg object
   * This this "IMU Message Format".
   * IMU data: acc+gyr at 1000Hz
   * Command: 75 65 0C 0A 0A 08 01 02 04 00 01 05 00 01 10 73
   * reply: 
   */
  AccGyr1kHzMsg(): GX5ImuMsg()
  {
    descr_set_ = 0x0C;
    cmd_field_data_.resize(10);
    cmd_field_data_[0] = static_cast<uint8_t>(cmd_field_data_.size());
    cmd_field_data_[1] = 0x08; // Field descr => command byte? (see answer)
    cmd_field_data_[2] = 0x01; // func use new settings
    cmd_field_data_[3] = 0x02; // 2 Descr
    cmd_field_data_[4] = 0x04; // 1rst Descr accelerometer
    cmd_field_data_[5] = 0x00; // Rate Dec ...
    cmd_field_data_[6] = 0x01; // ... 1kHz
    cmd_field_data_[7] = 0x05; // 2nd Descr gyrometer
    cmd_field_data_[8] = 0x00; // Rate Dec ...
    cmd_field_data_[9] = 0x01; // ... 1kHz
    construct_command();
    std::vector<uint8_t> expected_command = 
      {0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x08, 0x01, 0x02, 0x04, 0x00,
       0x01, 0x05, 0x00, 0x01, 0x10, 0x73};
    assert(command_ == expected_command && "\"IMU Message Format\" command wrong");

    reply_field_data_.resize(4);
    reply_field_data_[0] = static_cast<uint8_t>(reply_field_data_.size());
    reply_field_data_[1] = 0xF1; // field descr
    reply_field_data_[2] = 0x08; // echo the command byte
    reply_field_data_[3] = 0x00;
    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x0C, 0x04, 0x04, 0xF1, 0x08, 0x00, 0xE7, 0xBA};
    assert(expected_reply_ == expected_reply && "\"IMU Message Format\" expected reply wrong");

    reply_.resize(expected_reply_.size());
  }
};

/**
 * @brief IMU data: acc+gyr+quat at 1000Hz
 */
class AccGyrQuat1kHzMsg: public GX5ImuMsg
{
public:

  /**
   * @brief Construct a new AccGyr1kHzMsg object
   * This this "IMU Message Format".
   * IMU data: acc+gyr at 1000Hz
   * Command: 75 65 0C 0A 0A 08 01 02 04 00 01 05 00 01 10 73
   * reply: 
   */
  AccGyrQuat1kHzMsg(): GX5ImuMsg()
  {
    descr_set_ = 0x0C;
    cmd_field_data_.resize(13);
    cmd_field_data_[0]  = static_cast<uint8_t>(cmd_field_data_.size());
    cmd_field_data_[1]  = 0x08; // Field descr => command byte? (see answer)
    cmd_field_data_[2]  = 0x01; // func use new settings
    cmd_field_data_[3]  = 0x03; // 3 Descr
    cmd_field_data_[4]  = 0x04; // 1rst Descr accelerometer
    cmd_field_data_[5]  = 0x00; // Rate Dec ...
    cmd_field_data_[6]  = 0x01; // ... 1kHz
    cmd_field_data_[7]  = 0x05; // 2nd Descr gyrometer
    cmd_field_data_[8]  = 0x00; // Rate Dec ...
    cmd_field_data_[9]  = 0x01; // ... 1kHz
    cmd_field_data_[10] = 0x0A; // 3rd Descr gyrometer
    cmd_field_data_[11] = 0x00; // Rate Dec ...
    cmd_field_data_[12] = 0x01; // ... 1kHz
    construct_command();
    std::vector<uint8_t> expected_command = 
      {0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x08, 0x01, 0x02, 0x04, 0x00,
       0x01, 0x05, 0x00, 0x01, 0x0C, 0x00, 0x01, 0x10, 0x73};
    assert(command_ == expected_command && "\"IMU Message Format\" command wrong");

    reply_field_data_.resize(4);
    reply_field_data_[0] = static_cast<uint8_t>(reply_field_data_.size());
    reply_field_data_[1] = 0xF1; // field descr
    reply_field_data_[2] = 0x08; // echo the command byte
    reply_field_data_[3] = 0x00;
    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x0C, 0x04, 0x04, 0xF1, 0x08, 0x00, 0xE7, 0xBA};
    assert(expected_reply_ == expected_reply && "\"IMU Message Format\" expected reply wrong");

    reply_.resize(expected_reply_.size());
  }
};

/**
 * @brief This class allows us to get the EF data: RPY at 500Hz (max)
 * command: 75 65 0C 07 07 0A 01 01 05 00 01 06 23
 */
class EFdata500HzMsg: public GX5ImuMsg
{
public:
  /**
   * @brief 
   * 
   */
  EFdata500HzMsg(): GX5ImuMsg()
  {
    descr_set_ = 0x0C;
    cmd_field_data_.resize(7);
    cmd_field_data_[0] = static_cast<uint8_t>(cmd_field_data_.size());
    cmd_field_data_[1] = 0x0A; // Field descr
    cmd_field_data_[2] = 0x01; // func: use n
    cmd_field_data_[3] = 0x01; // Desc count
    cmd_field_data_[4] = 0x05; // 1rst Descr rpy
    cmd_field_data_[5] = 0x00; // Rate Dec ...
    cmd_field_data_[6] = 0x01; // ...
    construct_command();
    std::vector<uint8_t> expected_command = 
      {0x75, 0x65, 0x0C, 0x07, 0x07, 0x0A, 0x01, 0x01, 0x05, 0x00, 0x01, 0x06,
       0x23};
    assert(command_ == expected_command && "\"IMU Message Format\" command wrong");

    reply_field_data_.resize(4);
    reply_field_data_[0] = static_cast<uint8_t>(reply_field_data_.size());
    reply_field_data_[1] = 0xF1;
    reply_field_data_[2] = 0x0A;
    reply_field_data_[3] = 0x00;
    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x0C, 0x04, 0x04, 0xF1, 0x0A, 0x00, 0xE9, 0xBE};
    assert(expected_reply_ == expected_reply && "\"IMU Message Format\" expected reply wrong");

    reply_.resize(expected_reply_.size());
  }
};

/**
 * @brief This message enables the data stream for IMU and EF (estimation filter)
 */
class StreamImuEfMsg: public GX5ImuMsg
{
public:
  /**
   * @brief Construct a new StreamImuEfMsg object
   */
  StreamImuEfMsg(): GX5ImuMsg()
  {
    descr_set_ = 0x0C;
    cmd_field_data_.resize(10);
    cmd_field_data_[0] = 0x05; // field length
    cmd_field_data_[1] = 0x11; // Field descr
    cmd_field_data_[2] = 0x01; // New settings
    cmd_field_data_[3] = 0x01; // IMU
    cmd_field_data_[4] = 0x01; // start stream
    cmd_field_data_[5] = 0x05; // field length
    cmd_field_data_[6] = 0x11; // Field descr
    cmd_field_data_[7] = 0x01; // New settings
    cmd_field_data_[8] = 0x03; // Estimation Filter
    cmd_field_data_[9] = 0x01; // start stream
    construct_command();
    std::vector<uint8_t> expected_command = 
      {0x75, 0x65, 0x0C, 0x0A, 0x05, 0x11, 0x01, 0x01, 0x01, 0x05,
       0x11, 0x01, 0x03, 0x01, 0x24, 0xCC};
    assert(command_ == expected_command && "\"IMU Message Format\" command wrong");

    reply_field_data_.resize(8);
    reply_field_data_[0] = 0x04;
    reply_field_data_[1] = 0xF1; // reply description
    reply_field_data_[2] = 0x11; // I do not undesrtand why...
    reply_field_data_[3] = 0x00;
    reply_field_data_[4] = 0x04;
    reply_field_data_[5] = 0xF1; // reply description
    reply_field_data_[6] = 0x11; // I do not undesrtand why...
    reply_field_data_[7] = 0x00;
    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x0C, 0x08, 0x04, 0xF1, 0x11, 0x00, 0x04, 0xF1, 0x11, 0x00, 0xFA, 0xB5};
    assert(expected_reply_ == expected_reply && "\"IMU Message Format\" expected reply wrong");

    reply_.resize(expected_reply_.size());
  }
};

/**
 * @brief set heading at 0
 */
class SetHeading0Msg: public GX5ImuMsg
{
public:
  /**
   * @brief Construct a new SetHeading0Msg object
   */
  SetHeading0Msg(): GX5ImuMsg()
  {
    descr_set_ = 0x0D;
    cmd_field_data_.resize(6);
    cmd_field_data_[0] = 0x06; // field length
    cmd_field_data_[1] = 0x03; // Field descr (Yaw)
    cmd_field_data_[2] = 0x00; // Yaw value ...
    cmd_field_data_[3] = 0x00; // ...
    cmd_field_data_[4] = 0x00; // ...
    cmd_field_data_[5] = 0x00; // ... 0 rad
    construct_command();
    std::vector<uint8_t> expected_command = 
      {0x75, 0x65, 0x0D, 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF6, 0xE4};
    assert(command_ == expected_command && "\"Set EF yaw to 0\" command wrong");

    reply_field_data_.resize(4);
    reply_field_data_[0] = static_cast<uint8_t>(reply_field_data_.size());
    reply_field_data_[1] = 0xF1; // reply description
    reply_field_data_[2] = 0x03; // I do not undesrtand why...
    reply_field_data_[3] = 0x00;
    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x0D, 0x04, 0x04, 0xF1, 0x03, 0x00, 0xE3, 0xB6};
    assert(expected_reply_ == expected_reply && "\"Set EF yaw to 0\" expected reply wrong");

    reply_.resize(expected_reply_.size());
  }
};

/**
 * @brief Raw message containing the Estimation Filter data (roll, pitch, yaw)
 */
class ImuDataMsg: public GX5ImuMsg
{
public:
  ImuDataMsg(): GX5ImuMsg()
  {
    descr_set_ = 0x80;
    command_.resize(3);
    command_[0] = 0x75;
    command_[1] = 0x65;
    command_[2] = descr_set_;

    reply_field_data_.resize(46, 0x00);
    reply_field_data_[0] = 0x0E; // acc field length: 14 bytes
    reply_field_data_[1] = 0x04; // Acc, reply description
    
    reply_field_data_[14] = 0x0E; // gyro field length: 14 bytes
    reply_field_data_[15] = 0x05; // Gyro

    reply_field_data_[28] = 0x12; // quat field length: 18 bytes
    reply_field_data_[29] = 0x0A; // quat

    construct_expected_reply();
    std::vector<uint8_t> expected_reply = 
      {0x75, 0x65, 0x80, 0x1C, 
      0x0E, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x0E, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
      0x12, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
      0x9B, 0x00};    
    assert(expected_reply_ == expected_reply && "\"ImuDataMsg\" expected reply wrong");
    
    reply_.resize(52);
  }
};

/**
 * @brief Raw message containing the Estimation Filter data (roll, pitch, yaw)
 */
class EfDataMsg: public ImuMsg
{
public:
  EfDataMsg(): ImuMsg()
  {
    command_.clear();
    reply_.resize(56-34);
  }
};

} // namespace imu_3DM_GX5_25
} // namespace imu_core
#endif // IMU_3DM_GX3_25_MSG_HPP
