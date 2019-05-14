#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include <iomanip>
#include <sstream>
#include <vector>
#include <Eigen/Eigen>

namespace imu_core{

/**
 * @brief This class allow the user to write or read some msg.
 */
class ImuMsg
{
public:
  /**
   * @brief Construct a new ImuMsg object.
   */
  ImuMsg();
  /**
   * @brief Destroy the ImuMsg object.
   */
  ~ImuMsg(){}
  /**
   * @brief Check if the command_ and the reply_ objects has been initialized.
   */
  bool is_valid();
public:
  /**
   * @brief Contain the command to be written or that has been written
   */
  std::vector<uint8_t> command_;
  /**
   * @brief contain the device reply
   */
  std::vector<uint8_t> reply_;
};

/**
 * @brief This enum define the data streamed by the IMU upon setting the
 * continuous mode.
 */
enum ImuMsgTypes{
  /**
   * @brief The IMU broadcasts the acceleration and the angular rate.
   * This is the strongly recommanded interface.
   */
  Acceleration_AngularRate = 0,
  /**
   * @brief The IMU broadcasts the stabilized acceleration, the angular rate
   * and the magnetometer measurement.
   */
  StabilizedAcceleration_AngularRate_Magnetometer,
  /**
   * @brief The IMU broadcasts the acceleration, the angular rate and the
   * rotation matrix. The orientation is computed using an Extended Kalman
   * Filter integrated in the hardware. (Not recommanded).
   */
  Acceleration_AngularRate_OrientationMatrix,
  /**
   * @brief The IMU broadcasts the quaternion representation the IMU attitude
   * through the computation an Extended Kalman Filter integrated in the
   * hardware. (Not recommanded).
   */
  Quaternion
};

/**
 * @brief This interface propose an easy acces to the IMU data though Eigen.
 */
class ImuInterface {
public:
  /**
   * @brief Construct a new ImuInterface object
   * 
   * @param port_name is the device path in linux: e.g. /dev/tty0
   * @param stream_data is if the user wants to stream the data or poll them.
   */
  ImuInterface(const std::string port_name, bool stream_data);
  /**
   * @brief Destroy the ImuInterface object
   */
  virtual ~ImuInterface(void);
  /**
   * @brief Initialization method that will depend on each IMU.
   * 
   * @return true if success.
   * @return false is failure.
   */
  virtual bool initialize() = 0;
  /**
   * @brief Get the acceleration_ object
   * 
   * @return const Eigen::Ref<Eigen::Vector3d> 
   */
  const Eigen::Ref<Eigen::Vector3d> get_acceleration()
  {
    return acceleration_;
  }
  /**
   * @brief Get the angular_rate_ object
   * 
   * @return const Eigen::Ref<Eigen::Vector3d> 
   */
  const Eigen::Ref<Eigen::Vector3d> get_angular_rate()
  {
    return angular_rate_;
  }

private:
  /**
   * @brief Measurement of the accelerometer
   */
  Eigen::Vector3d acceleration_;
  /**
   * @brief Measurement of the gyroscope.
   */
  Eigen::Vector3d angular_rate_;
  /**
   * @brief Name of the linux path, e.g. /dev/ttyACM0
   */
  std::string port_name_;
  /**
   * @brief Do we stream data or not.
   */
  bool stream_data_;
};

} // namespace
#endif // Header protection
