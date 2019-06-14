#ifndef IMU_3DM_GX3_25_MSG_HPP
#define IMU_3DM_GX3_25_MSG_HPP

#include "imu-core/imu_interface.hpp"

namespace imu_core
{
namespace imu_3DM_GX3_25
{

  /**
   * @brief This structure defines the data type streamed by the IMU upon
   * setting the continuous mode.
   */
  struct DataType{
    /**
     * @brief The IMU broadcasts the acceleration and the angular rate.
     * This is the strongly recommanded interface.
     */
    static const uint8_t AccGyro = 0xc2;
    /**
     * @brief The IMU broadcasts the stabilized acceleration, the angular rate
     * and the magnetometer measurement.
     */
    static const uint8_t StabAccGyroMagn = 0xd2;
    /**
     * @brief The IMU broadcasts the acceleration, the angular rate and the
     * rotation matrix. The orientation is computed using an Extended Kalman
     * Filter integrated in the hardware. (Not recommanded).
     */
    static const uint8_t AccGyroRotMat = 0xc8;
    /**
     * @brief The IMU broadcasts the quaternion representation the IMU attitude
     * through the computation an Extended Kalman Filter integrated in the
     * hardware. (Not recommanded).
     */
    static const uint8_t Quaternion = 0xdf;

    static bool check_data_type(uint8_t data_type)
    {
      return (data_type == AccGyro || data_type == StabAccGyroMagn ||
              data_type == AccGyroRotMat || data_type == Quaternion);
    }
  };

/**
 * @brief This class allows us to send a simple message to the IMU in order to
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

    /**
     * Reply
     */
    reply_.resize(10, 0);
  }
};

/**
 * @brief This class allows us to send a simple message to the IMU in order to
 * modify its data sampling settings.
 */
class SamplingSettingsMsg: public ImuMsg
{
public:

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
    command_[3] = 1 ; // change value temporarily;

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
    *(uint32_t *)(&command_[4]) = ImuInterface::bswap_16(data_rate_decimation);

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
    // assert(1 <= gyro_acc_window_filter_divider &&
    //        gyro_acc_window_filter_divider <= 32 &&
    //        "gyro acc filter divider must be in [1, 32]");
    command_[8] = gyro_acc_window_filter_divider;

    /**
     * Mag digital filter window size.  First null is 1000 divided by this
     * value. Minimum value is 1, maximum value is 32  Default is 17.
     */
    uint8_t magn_window_filter_divider = 17;
    // assert(1 <= magn_window_filter_divider &&
    //        magn_window_filter_divider <= 32 &&
    //        "magn filter divider must be in [1, 32]");
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
     * Reply
     */
    reply_.resize(19, 0);
  }
};

/**
 * @brief This class allows us to (re)set or receive the onboard time stamp.
 */
class TimerMsg: public ImuMsg
{
public:

  /** 
   * @brief Function selector: (8bit unsigned integer)
   * 0:  Do not change the time stamp, just return current value
   * 1:  Restart the time stamp at the new value
   * 2:  Restart the PPS Seconds counter at the new value
   */
  enum ChangeParam {
    ReturnCurrent = 0,
    RestartTimerAtNewValue = 1,
    RestartPPSsecCounterAtNewValue = 2
  };
  /**
   * @brief Construct a new TimerMsg object
   */
  TimerMsg(): ImuMsg()
  {
    // Reset onboard timestamp:
    command_.resize(8);
    command_[0] = 0xd7 ; // header
    command_[1] = 0xc1 ; // user confirmation 1
    command_[2] = 0x29 ; // user confirmation 2
    command_[3] = (uint8_t)ChangeParam::RestartTimerAtNewValue ;
    *(uint32_t *)(&command_[4]) = ImuInterface::bswap_32(0); // start at 0

    /**
     * Reply
     */
    reply_.resize(7, 0);
  }
};

/**
 * @brief This class allows us to compute the gyro bias after a certain time.
 * I assume this period of time is used by the onboard (extended kalman) filter
 * in order to compute the Bias.
 */
class CaptureGyroBiasMsg: public ImuMsg
{
public:
  /**
   * @brief Construct a new CaptureGyroBiasMsg object
   * 
   * @param calibration_duration is the duration given to the IMU to compute the
   * gyroscope bias.
   */
  CaptureGyroBiasMsg(uint16_t calibration_duration = 3): ImuMsg()
  {
    // Record gyroscope biases:
    command_.resize(5);
    command_[0] = 0xcd ; // header
    command_[1] = 0xc1 ; // user confirmation 1
    command_[2] = 0x29 ; // user confirmation 2
    // by default we wait 3seconds
    *(uint32_t *)(&command_[3]) = ImuInterface::bswap_32(calibration_duration);

    /**
     * Reply is of size 19
     */
    reply_.resize(19);
  }
};

/**
 * @brief This class allows us to ask for the data steam to start.
 */
class StartDataStreamMsg: public ImuMsg
{
public:
  /**
   * @brief Construct a new StartDataStreamMsg object
   * 
   * @param data_type define which data we receive from the IMU:
   * + AccGyro
   * + StabAccGyroMagn
   * + AccGyroRotMat
   * + Quaternion
   * See the DataType struct for more details.
   */
  StartDataStreamMsg(uint8_t data_type = DataType::AccGyro): ImuMsg()
  {
    assert(DataType::check_data_type(data_type) &&
           "This data type is not supported.");
    // Set continuous mode:
    command_.resize(4);
    command_[0] = 0xc4 ; // header
    command_[1] = 0xc1 ; // user confirmation 1
    command_[2] = 0x29 ; // user confirmation 2
    command_[3] = DataType::AccGyro;

    /**
     * Reply
     */
    reply_.resize(8);
  }
};

/**
 * @brief This class allows us to ask for the data steam to stop.
 */
class StopDataStreamMsg: public ImuMsg
{
public:
  /**
   * @brief Construct a new StopDataStreamMsg object
   */
  StopDataStreamMsg(): ImuMsg()
  {
    if(0) // reuse the set continuous mode message (get a reply)
    {
      // Set continuous mode:
      command_.resize(4);
      command_[0] = 0xc4 ; // header
      command_[1] = 0xc1 ; // user confirmation 1
      command_[2] = 0x29 ; // user confirmation 2
      command_[3] = 0; // here we deactivate the continuous mode with this 0
      /**
       * Reply
       */
      reply_.resize(8);
    }else // use the stop continuous mode message (No reply)
    {
      // Stop continuous mode:
      command_.resize(3);
      command_[0] = 0xfa ; // header
      command_[1] = 0x75 ; // user confirmation 1
      command_[2] = 0xb4 ; // user confirmation 2
      /**
       * No reply
       */
      reply_.clear();
    }
  }
};

/**
 * @brief Message requesting the soft reset of the imu.
 */
class ResetMsg: public ImuMsg
{
public:
  ResetMsg(): ImuMsg()
  {
    // Acceleration and angular rate:
    command_.resize(3);
    command_[0] = 0xfa;
    command_[0] = 0x75;
    command_[0] = 0xb4;
    /**
     * No reply
     */
    reply_.clear();
  }
};

/**
 * @brief Message requesting the Accelerometer and Gyroscope measurements.
 */
class AccGyroMsg: public ImuMsg
{
public:
  AccGyroMsg(): ImuMsg()
  {
    // Acceleration and angular rate:
    command_.resize(1);
    command_[0] = 0xc2;
    /**
     * Reply
     */
    reply_.resize(31);
  }
};

/**
 * @brief Message requesting the Stabilized Accelerometer, Gyroscope and
 * Magnetometer measurements.
 */
class StabAccGyroMagnMsg: public ImuMsg
{
public:
  StabAccGyroMagnMsg(): ImuMsg()
  {
    // Stabilized acceleration, angular rate and magnetometer:
    command_.resize(1);
    command_[0] = 0xd2;
    /**
     * Reply
     */
    reply_.resize(43);
  }
};

/**
 * @brief Message requesting the Accelerometer, Gyroscope and rotation matrix
 * measurements.
 */
class AccGyroRotMatMsg: public ImuMsg
{
public:
  AccGyroRotMatMsg(): ImuMsg()
  {
    // Acceleration, angular rate and orientation matrix:
    command_.resize(1);
    command_[0] = 0xc8;
    /**
     * Reply
     */
    reply_.resize(67);
  }
};

/**
 * @brief Message requesting the quaternion measurement.
 */
class QuaternionMsg: public ImuMsg
{
public:
  QuaternionMsg(): ImuMsg()
  {
    // Quaternion:
    command_.resize(1);
    command_[0] = 0xdf;
    /**
     * Reply
     */
    reply_.resize(23);
  }
};

} // namespace imu_3DM_GX3_25
} // namespace imu_core
#endif // IMU_3DM_GX3_25_MSG_HPP
