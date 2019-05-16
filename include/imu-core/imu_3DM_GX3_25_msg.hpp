#ifndef IMU_3DM_GX3_25_MSG_HPP
#define IMU_3DM_GX3_25_MSG_HPP

#include "imu-core/imu_interface.hpp"

namespace imu_core
{
namespace imu_3DM_GX3_25
{
/**
 * @brief This class allow us to send a simple message to the IMU in order to
 * modify its communication settings.
 */
class CommunicationSettingsMsg: public ImuMsg
{
public:
  /**
   * @brief this allow us to define if the value should return, changed, or
   * changed and stored.
   */
  enum ChangeParam{
    GetCurrentValue=0,
    ChangeValueTemporarily=1,
    ChangeValuePermanently=2,
  };

  /**
   * @brief Construct a new CommunicationSettingsMsg object
   * 
   * @param baude_rate_cst is the baude rate the imu can use based on the
   * available ones:
   *    BaudeRate::BR_115200
   *    BaudeRate::BR_230400
   *    BaudeRate::BR_460800
   *    BaudeRate::BR_921600
   * @param change_param allow the user to ask for permanent change of paramters
   * or not
   */
  CommunicationSettingsMsg(BaudeRate baude_rate_cst):
    ImuMsg()
  {
    command_.resize(11);
    command_[0] = 0xd9 ; // header
    command_[1] = 0xc3 ; // user confirmation 1
    command_[2] = 0x55 ; // user confirmation 2
    command_[3] = (uint8_t)1; // UART1, Primary UART for host communication
    command_[4] = (uint8_t)ChangeParam::ChangeValueTemporarily;
    uint32_t baud_rate = 0;
    switch (baude_rate_cst)
    {
    case BaudeRate::BR_115200:
      baud_rate = 115200;
      *(uint32_t *)(&command_[5]) = ImuInterface::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_230400:
      baud_rate = 230400;
      *(uint32_t *)(&command_[5]) = ImuInterface::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_460800:
      baud_rate = 460800;
      *(uint32_t *)(&command_[5]) = ImuInterface::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_921600:
      baud_rate = 921600;
      *(uint32_t *)(&command_[5]) = ImuInterface::bswap_32(baud_rate);
      break;    
    default:
      throw std::runtime_error(
        "ImuMsgCommunicationSettings::ImuMsgCommunicationSettings() : "
        "Baude rate supported by the imu, fix the code or correct baude rate");
      break;
    }
    command_[9] = (uint8_t)2; // 2: Selected UART Enabled ; 0: Selected UART Disabled
    command_[10] = (uint8_t)0; // Reserved: Set to 0
    reply_.resize(10);
  }
};

/**
 * @brief This class allow us to send a simple message to the IMU in order to
 * modify its data sampling settings.
 */
class SamplingSettingsMsg: public ImuMsg
{
public:
  /**
   * @brief this allow us to define if the value should return, changed, or
   * changed and stored.
   */
  enum ChangeParam{
    GetCurrentValue=0,
    ChangeValueTemporarily=1,
    ChangeValueStoreVolatile=2,
    ChangeValueNoReply=3,
  };

  /**
   * @brief Construct a new SamplingSettingsMsg object
   * 
   * @param data_rate_decimation this divides the maximum rate of data (1000Hz).
   *   So if data_rate_decimation = 1, the rate of data is 1000Hz.
   *   And if data_rate_decimation = 1000, the rate of data is 1Hz.
   * @param change_param allow the user to ask for permanent change of paramters
   *   or not
   * @param gyro_acc_window_filter_divider is defining the Gyrometer and
   *   Accelerometer filter window size. The size is
   *   (1000/gyro_acc_window_filter_divider). The default value is (1000/15).
   *   1 <= gyro_acc_window_filter_divider <= 32
   * @param magn_window_filter_divider is defining the Magnetometer filter 
   *   window size. The size is (1000/magn_window_filter_divider). The default 
   *   value is (1000/17). 1 <= magn_window_filter_divider <= 32
   */
  SamplingSettingsMsg(): ImuMsg()
  {
    command_.resize(20);
    command_[0] = 0xdb ; // header
    command_[1] = 0xa8 ; // user confirmation 1
    command_[2] = 0xb9 ; // user confirmation 2
    command_[3] = (uint8_t)ChangeParam::ChangeValueTemporarily;

    /**
     * Data Rate decimation value.  This valueisdivided intoa fixed 1000Hz
     * reference ratetoestablish the data output rate.  Settingthis value to 10
     * gives an output data rate of 1000/10= 100 samples/sec.  When using the 
     * UART for communications, atdata rates higher than 250, the UART baud 
     * rate must be increased (see command 0xD9).Minimum Value is 1, Maximum 
     * value is 1000.
     */
    uint16_t data_rate_decimation = 1;
    assert(data_rate_decimation <= 1000 && data_rate_decimation >= 1 && 
           "The data rate decimation must be in [1 ; 1000]");
    *(uint32_t *)(&command_[4]) = ImuInterface::bswap_32(data_rate_decimation);

    /**
     * Data conditioning function selector:
     *    Bit 0:  if set -Calculate orientation.  Default is “1”
     *    Bit 1:  if set -Enable Coning&Sculling.  Default is “1”
     *    Bit 2 –3:  reserved.  Default is “0”
     *    Bit 4:  if set –Floating Pointdata is sent in Little Endian format 
     *            (only floating point data from IMU to HOST is affected).
     *            Default is “0”
     *    Bit 5:  if set –NaN data is suppressed.  Default is “0”
     *    Bit 6:  if set, enable finite size correctionDefault is “0”
     *    Bit 7:  reserved.  Default is “0”
     *    Bit 8:  if set, disables magnetometerDefault is “0”
     *    Bit 9:  reserved.  Default is “0”
     *    Bit 10:  if set, disables magnetic north compensationDefault is “0”
     *    Bit 11:  if set, disables gravity compensation  Default is “0”
     *    Bit 12:  if set, enables Quaternion calculation  Default is “0”
     *    Bit 13–15:  reserved.  Default is “0”
     * We select uniquely the accelerometer and gyroscope data.
     */
    uint16_t fselect = 0b0000110100010000; // get the acc and gyro
    // fselect = 0b0001010100010011; // calculate quaternion, needs 500Hz
    // fselect = 0b0000010100010011; // calculate rotation matrix, needs 500Hz
    *(uint16_t *)(&command_[6]) = ImuInterface::bswap_16(fselect);

    /**
     * Gyro and Accel digital filter window size. First null is 1000 divided by
     * this value. Minimum value is 1, maximum value is 32. Default is 15
     */
    uint8_t gyro_acc_window_filter_divider = 15;
    assert(1 <= gyro_acc_window_filter_divider &&
           gyro_acc_window_filter_divider <= 32 &&
           "gyro acc filter divider must be in [1, 32]");
    command_[8] = gyro_acc_window_filter_divider;

    /**
     * Mag digital filter window size.  First null is 1000 divided by this
     * value. Minimum value is 1, maximum value is 32  Default is 17.
     */
    uint8_t magn_window_filter_divider = 17;
    assert(1 <= magn_window_filter_divider &&
           magn_window_filter_divider <= 32 &&
           "magn filter divider must be in [1, 32]");
    command_[9] = magn_window_filter_divider;

    /**
     * Up Compensation in seconds. Determines how quickly the gravitational 
     * vector corrects the gyro stabilized pitch and roll. Minimum value is 1, 
     * maximum value is 1000.Default is 10
     */
    command_[10] = 0;
    command_[11] = 10;

    /**
     * NorthCompensation in seconds. Determines how quickly the magnetometer 
     * vector corrects the gyro stabilized yaw. Minimum value is 1, maximum
     * value is 1000.Default is 10
     */
    command_[12] = 0;
    command_[13] = 10;
    /**
     * Mag Power/Bandwidth setting.
     *     0: Highest bandwidth, highest power.
     *     1: Lowerpower, bandwidthcoupled to data rate
     */
    command_[14] = 0;
    /**
     * reserved bytes to be all zeros
     */
    command_[15] = 0;
    command_[16] = 0;
    command_[17] = 0;
    command_[18] = 0;
    command_[19] = 0;

    /**
     * Reply is of size 19
     */
    reply_.resize(19);
  }
};

} // namespace imu_3DM_GX3_25
} // namespace imu_core
#endif // IMU_3DM_GX3_25_MSG_HPP
