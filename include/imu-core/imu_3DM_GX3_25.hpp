#ifndef IMU_3DM_GX3_25_HPP
#define IMU_3DM_GX3_25_HPP

#include "real_time_tools/thread.hpp"
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/mutex.hpp"
#include "real_time_tools/usb_stream.hpp"
#include "real_time_tools/timer.hpp"

#include "imu-core/imu_interface.hpp"

namespace imu_core
{
namespace imu_3DM_GX3_25
{

/**
 * @brief This class correspond to the driver of the imu 3DM_GX3_25 from
 * MicroStrain. Official API based on the common interface for IMUS.
 * 
 */
class Imu3DM_GX3_25: public ImuInterface
{
/**
 * Public interface
 */
public:
  /**
   * @brief Construct a new Imu3DM_GX3_25 object
   * 
   * @param port_name  is the linux device name, e.g. "/dev/ttyACM0".
   */
  Imu3DM_GX3_25(const std::string& port_name);

  /**
   * @brief Destroy the Imu3DM_GX3_25 object
   */
  virtual ~Imu3DM_GX3_25(void);

  /**
   * @brief Inherted method from the ImuInterface. It launch the thread to
   * stream the data and initialize the usb communication.
   * 
   * @return true if success
   * @return false if failure
   */
  bool initialize();
  
  /**
   * Helper methods. The only public methods the user should use are
   * the one defined by the ImuInterface.
   */

  /**
   * @brief Read a message form the IMU and check the alignement, the checksum
   * and the header.
   * 
   * @param msg the message conatining the command sent and the bufffer to
   * store the message.
   * @return true if success
   * @return false if failure
   */
  bool receive_message(ImuMsg& msg, bool stream_mode);

  /**
   * @brief Check if the received information has the correct checksum.
   * 
   * @param msg is the ImuMsg containing the reponse and the command message.
   * @return true 
   * @return false 
   */
  bool is_checksum_correct(const ImuMsg& msg);

  /**
   * @brief Realign the reading loop to misaligned messages.
   * 
   * @param msg the msg command and reply
   * @return true if success
   * @return false if failure
   */
  bool read_misaligned_msg_from_device(ImuMsg& msg);

  /**
   * @brief Send a message to the IMU
   * 
   * @param msg the message to be sent
   * @return true if success
   * @return false if failure
   */
  bool send_message(const ImuMsg& msg)
  {
    return usb_stream_.write_device(msg.command_);
  }

  /**
   * @brief Set the communication settings of the IMU
   * 
   * @return true if success
   * @return false if failure
   */
  bool set_communication_settings(void);

  /**
   * @brief Set the sampling settings of the IMU data
   * 
   * @return true if success
   * @return false if failure
   */
  bool set_sampling_settings(void);

  /**
   * @brief initialize the internal IMU time stamp to the PC one.
   * 
   * @return true if success
   * @return false if failure
   */
  bool initialize_time_stamp(void);

  /**
   * @brief Wait 3 seconds and get the gyroscope bias.
   * 
   * @return true if success
   * @return false if failure
   */
  bool capture_gyro_bias(void);
  
  /**
   * @brief This helper function allows us to start the thread.
   * 
   * @param object is the current IMU object
   * @return THREAD_FUNCTION_RETURN_TYPE
   */
  static THREAD_FUNCTION_RETURN_TYPE reading_loop_helper(void* object)
  {
    Imu3DM_GX3_25* imu_object = static_cast<Imu3DM_GX3_25*>(object);
    if(imu_object->reading_loop())
    {
      rt_printf("Imu3DM_GX3_25::reading_loop_helper(): [Status] "
                "thread closing normally.");
    }else{
      rt_printf("Imu3DM_GX3_25::reading_loop_helper(): [Error] "
                "thread closing after an error occured.");
    }
  }

  /**
   * @brief This is the sensor acquisition loop
   * 
   * @return true if success
   * @return false if failure
   */
  bool reading_loop(void);

  /**
   * @brief Flag the reading loop to stop its activity and join the thread
   * 
   * @return true if success
   * @return false if failure
   */
  bool stop_reading_loop(void);

  /**
   * @brief Utilities to swap 16 bits
   * 
   * @param x 
   * @return uint16_t 
   */
  static inline uint16_t bswap_16(uint16_t x){
    return (x >> 8) | (x << 8);
  }

  /**
   * @brief Utilities to swap 32 bits
   * 
   * @param x 
   * @return uint16_t 
   */
  static inline uint16_t bswap_32(uint16_t x)
  {
    return (bswap_16(x & 0xffff) << 16) | (bswap_16(x >> 16));
  }

private:
  /**
   * @brief Create a real time thread.
   */
  real_time_tools::RealTimeThread thread_;
  /**
   * @brief Manage real time mutexes.
   */
  real_time_tools::RealTimeMutex mutex_;
  /**
   * @brief Manage the real time communication with the device.
   */
  real_time_tools::UsbStream usb_stream_;
  /**
   * @brief Computer port configuration
   */
  real_time_tools::PortConfig port_config_;
  /**
   * @brief Maximum number of try while realigning the data stream.
   */
  int max_realign_trials_;
  /**
   * @brief Measure the performance of the reading loop.
   */
  real_time_tools::Timer timer_;
  /**
   * @brief Get the data time stamp
   */
  double time_stamp_;
};


/**
 * @brief Simple renaming for conveniency.
 */
typedef real_time_tools::PortConfig::BaudeRate BaudeRate;


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
   * @param function_selector 
   */
  CommunicationSettingsMsg(BaudeRate baude_rate_cst,
                           ChangeParam function_selector =
                            ChangeParam::ChangeValueTemporarily):
    ImuMsg()
  {
    command_.resize(11);
    command_[0] = 0xd9 ; // header
    command_[1] = 0xc3 ; // user confirmation 1
    command_[2] = 0x55 ; // user confirmation 2
    command_[3] = (uint8_t)1; // UART1, Primary UART for host communication
    command_[4] = (uint8_t)function_selector;
    uint32_t baud_rate = 0;
    switch (baude_rate_cst)
    {
    case BaudeRate::BR_115200:
      baud_rate = 115200;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_230400:
      baud_rate = 230400;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_460800:
      baud_rate = 460800;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_921600:
      baud_rate = 921600;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
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

// Set sampling settings:
#define CMD_SAMP_SETTINGS 0xdb
#define CMD_SAMP_SETTINGS_LEN 20

#define SAMP_SETTINGS_CONF1 0xa8
#define SAMP_SETTINGS_CONF2 0xb9
#define RPLY_SAMP_SETTINGS_LEN 19
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
   * So if data_rate_decimation = 1, the rate of data is 1000Hz.
   * And if data_rate_decimation = 1000, the rate of data is 1Hz.
   * 
   * @param function_selector 
   */
  SamplingSettingsMsg(uint16_t data_rate_decimation = 1,
                      ChangeParam function_selector = 
                        ChangeParam::ChangeValueTemporarily):
    ImuMsg()
  {
    command_.resize(11);
    command_[0] = 0xdb ; // header
    command_[1] = 0xa8 ; // user confirmation 1
    command_[2] = 0xb9 ; // user confirmation 2
    command_[3] = (uint8_t)function_selector;
    assert(data_rate_decimation <= 1000 && data_rate_decimation >= 1 && 
           "The data rate decimation must be in [1 ; 1000]");
    *(uint32_t *)(&command_[4]) = Imu3DM_GX3_25::bswap_32(data_rate_decimation);
    
    
    *(uint16_t *)(&buffer_[6]) = bswap_16(fselect);
    
    uint32_t baud_rate = 0;
    switch (baude_rate_cst)
    {
    case BaudeRate::BR_115200:
      baud_rate = 115200;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_230400:
      baud_rate = 230400;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_460800:
      baud_rate = 460800;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
      break;
    case BaudeRate::BR_921600:
      baud_rate = 921600;
      *(uint32_t *)(&command_[5]) = Imu3DM_GX3_25::bswap_32(baud_rate);
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

} // namespace imu_3DM_GX3_25
} // namespace imu_core
#endif // IMU_3DM_GX3_25_HPP
