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
   * @brief Send the message to the IMU to start streaming data
   * 
   * @return true if success
   * @return false if failure
   */
  bool start_streaming_data(void);

  /**
   * @brief Send the message to the IMU to stop streaming data
   * 
   * @return true if success
   * @return false if failure
   */
  bool stop_streaming_data(void);

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
   * @brief Parse the accelerometer and gyroscope from the message
   * 
   * @return true 
   * @return false 
   */
  bool receive_acc_gyro(bool stream_data);

  /**
   * @brief Parse the stabilized accelerometer, gyroscope and magnetometer
   * from the message
   * 
   * @return true 
   * @return false 
   */
  bool receive_stab_acc_gyro_magn(bool stream_data){}

  /**
   * @brief Parse the accelerometer, gyroscope and rotation matrix from the
   * message
   * 
   * @return true 
   * @return false 
   */
  bool receive_acc_gyro_rot_mat(bool stream_data){}

  /**
   * @brief Parse the quaternion from the message
   * 
   * @return true 
   * @return false 
   */
  bool receive_quaternion(bool stream_data){}

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
  /**
   * @brief Manage the thread.
   */
  bool stop_imu_communication_;
};

} // namespace imu_3DM_GX3_25
} // namespace imu_core
#endif // IMU_3DM_GX3_25_HPP
