/**
 * @file imu_3DM_GX5_25.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-09-13
 * 
 * Generic interface for the 3DM-GX5-25 IMU
 */

#ifndef IMU_3DM_GX3_25_HPP
#define IMU_3DM_GX3_25_HPP

#include "real_time_tools/thread.hpp"
#include "real_time_tools/iostream.hpp"
#include "real_time_tools/mutex.hpp"
#include "real_time_tools/usb_stream.hpp"
#include "real_time_tools/timer.hpp"

#include "imu-core/imu_interface.hpp"
#include "imu-core/imu_3DM_GX5_25_msg.hpp"

#define DEBUG_PRINT_IMU_GX5_25 false

namespace imu_core
{
namespace imu_3DM_GX5_25
{

enum AccIndex
{
  index_acc_x = 6,
  index_acc_y = 10,
  index_acc_z = 14
};

enum GyroIndex
{
  index_gyro_x = 20,
  index_gyro_y = 24,
  index_gyro_z = 28
};

enum RpyIndex
{
  index_rpy_x = 6,
  index_rpy_y = 10,
  index_rpy_z = 14
};

/**
 * @brief This class correspond to the driver of the imu 3DM_GX3_25 from
 * MicroStrain. Official API based on the common interface for IMUS.
 * 
 */
class Imu3DM_GX5_25: public ImuInterface
{
/**
 * Public interface
 */
public:
  /**
   * @brief Construct a new Imu3DM_GX5_25 object
   * 
   * @param port_name  is the linux device name, e.g. "/dev/ttyACM0".
   * @param stream_data defines if the imu should stream its data or if we
   * systematically ask for them.
   */
  Imu3DM_GX5_25(const std::string& port_name,
                const bool& stream_data=false);

  /**
   * @brief Destroy the Imu3DM_GX5_25 object
   */
  virtual ~Imu3DM_GX5_25(void);

  /**
   * @brief Inherted method from the ImuInterface. It launch the thread to
   * stream the data and initialize the usb communication.
   * 
   * @return true if success
   * @return false if failure
   */
  virtual bool initialize();
  
  /**
   * Helper methods. The only public methods the user should use are
   * the one defined by the ImuInterface.
   */
    
  /**
   * @brief Open the communication and setup the default communication rate
   * 
   * @return true if success 
   * @return false if failure 
   */
  bool open_usb_port(int bauderate=115200);

  /**
   * @brief Read a message form the IMU and check the alignement, the checksum
   * and the header.
   * 
   * @param msg the message conatining the command sent and the bufffer to
   * store the message.
   * @param stream_mode defines if we simply read the port, or if we poll.
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
    bool success = true;
    success = success && usb_stream_.write_device(msg.command_);
    return success;
  }

  /**
   * @brief Set the imu into an idle mode to ease the communication
   * 
   * @return true 
   * @return false 
   */
  bool idle_mode();

  /**
   * @brief Set the IMU to send the imu (accelerometer and gyroscope) data
   * at 1 kHz.
   * 
   * @return true 
   * @return false 
   */
  bool imu_data_1kHz();

  /**
   * @brief Set the IMU to send the estimator filter (position and angle) data
   * at 0.5 kHz.
   * 
   */
  bool estimation_filter_data_500Hz();
  
  /**
   * @brief Send the message to the IMU to start streaming data
   * 
   * @return true if success
   * @return false if failure
   */
  bool start_streaming_data(void);

  /**
   * @brief Set the msg heading at 0
   * 
   * @return true
   * @return false
   */
  bool set_heading_at_0();

  /**
   * @brief This helper function allows us to start the thread.
   * 
   * @param object is the current IMU object
   * @return THREAD_FUNCTION_RETURN_TYPE
   */
  static THREAD_FUNCTION_RETURN_TYPE reading_loop_helper(void* object)
  {
    Imu3DM_GX5_25* imu_object = static_cast<Imu3DM_GX5_25*>(object);
    if(imu_object->reading_loop())
    {
      if(DEBUG_PRINT_IMU_GX5_25)
      {
        rt_printf("Imu3DM_GX5_25::reading_loop_helper(): [Status] "
                  "thread closing normally.\n");
      }
    }else{
      if(!imu_object->stop_imu_communication_)
      {
        if(DEBUG_PRINT_IMU_GX5_25)
        {
          rt_printf("Imu3DM_GX5_25::reading_loop_helper(): [Error] "
                      "thread closing after an error occured.\n");
        }
      }
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
  bool receive_data(bool stream_data);

  /**
   * @brief Parse the stabilized accelerometer, gyroscope and magnetometer
   * from the message
   * 
   * @return true 
   * @return false 
   */
  bool receive_estimator_filter_data(bool stream_data){}

  /**
   * @brief Convert 4 bits in a float number
   * 
   * @return double
   */
  double double_from_byte_array(std::vector<uint8_t> buffer, unsigned start_index)
  {
    assert(buffer.size() > start_index + 3 && "Imu3DM_GX5_25::double_from_byte_array The buffer is too small");
    union double_int {
      float f;
      unsigned long ul;
    } data;
    data.ul = ((buffer[start_index] << 24) | (buffer[start_index + 1] << 16) |
              (buffer[start_index + 2] << 8) | (buffer[start_index + 3]));
    return static_cast<double>(data.f);
  }

  void wait_data_aligned()
  {
    while(!aligned_data_)
    {
      real_time_tools::Timer::sleep_ms(5);
    }
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
  /**
   * @brief Manage the thread.
   */
  bool stop_imu_communication_;
  /**
   * @brief Define if the imu is streaming the data or if we use it in a poll
   * mode.
   */
  bool stream_data_;
  /**
   * @brief The raw data from the imu are stored here.
   */
  ImuDataMsg imu_data_msg_;
  /**
   * @brief Estimation filter raw data
   */
  EfDataMsg ef_data_msg_;
  /**
   * @brief Estimation filter data: Roll Pitch Yaw
   */
  Eigen::Vector3d rpy_;
  /**
   * @brief Check if the data is aligned or not, we wait until so.
   */
  bool aligned_data_;
};

} // namespace imu_3DM_GX3_25
} // namespace imu_core
#endif // IMU_3DM_GX3_25_HPP
