#ifndef IMU_3DM_GX3_25_HPP
#define IMU_3DM_GX3_25_HPP

#include "real_time_tools/realtime_iostream.hpp"
#include "real_time_tools/rt_mutex.hpp"
 
#include <stdint.h>
#include <stdio.h>
#include <termio.h>

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "imu-core/imu_interface.hpp"

/* SUPPORTED MESSAGE TYPES: */

// Acceleration and angular rate:
#define  CMD_AC_AN        0xc2
#define  CMD_AC_AN_LEN    1
#define  RPLY_AC_AN_LEN   31

// Stabilized acceleration, angular rate and magnetometer:
#define CMD_STAB_AC_AN_MAG        0xd2
#define CMD_STAB_AC_AN_MAG_LEN    1
#define RPLY_STAB_AC_AN_MAG_LEN   43

// Acceleration, angular rate and orientation matrix:
#define CMD_AC_AN_OR         0xc8
#define CMD_AC_AN_OR_LEN     1
#define RPLY_AC_AN_OR_LEN    67

// Quaternion:
#define CMD_QUAT       0xdf
#define CMD_QUAT_LEN   1
#define RPLY_QUAT_LEN  23

// Set continuous mode:
#define CMD_CONT_MODE       0xc4
#define CMD_CONT_MODE_LEN   4
#define CONT_MODE_CONF1     0xc1
#define CONT_MODE_CONF2     0x29
#define RPLY_CONT_MODE_LEN  8

// Stop continuous mode:
#define CMD_STOP_CONT       0xfa
#define CMD_STOP_CONT_LEN   3
#define STOP_CONT_CONF1     0x75
#define STOP_CONT_CONF2     0xb4

// Set sampling settings:
#define CMD_SAMP_SETTINGS               0xdb
#define  CMD_SAMP_SETTINGS_LEN          20
#define  SAMP_SETTINGS_CONF1            0xa8
#define  SAMP_SETTINGS_CONF2            0xb9
#define  RPLY_SAMP_SETTINGS_LEN         19

// Set communication settings:
#define CMD_COMM_SETTINGS               0xd9
#define  CMD_COMM_SETTINGS_LEN          11
#define  COMM_SETTINGS_CONF1            0xc3
#define  COMM_SETTINGS_CONF2            0x55
#define  RPLY_COMM_SETTINGS_LEN         10

// Record gyroscope biases:
#define  CMD_GYRO_BIAS                  0xcd
#define  CMD_GYRO_BIAS_LEN              5
#define  GYRO_BIAS_CONF1                0xc1
#define  GYRO_BIAS_CONF2                0x29
#define  RPLY_GYRO_BIAS_LEN             19

// Set onboard timestamp:
#define  CMD_TIMER                      0xd7
#define  CMD_TIMER_LEN                  8
#define  TIMER_CONF1                    0xc1
#define  TIMER_CONF2                    0x29
#define  RPLY_TIMER_LEN                 7

/**
 * @brief This class correspond to the driver of the imu 3DM_GX3_25 from
 * MicroStrain.
 */
class Imu3DM_GX3_25 {
/**
 * Official API based on the common interface for IMUS
 */
public:

    /**
     * @brief Construct a new Imu3DM_GX3_25 object
     * 
     * @param portname is the linux device name, e.g. "/dev/ttyACM0".
     * @param stream_data defines if the hardware is on poll or if it just
     * streams the data
     * @param real_time defines if we operate with a real_time operating system or not. @todo to be removed.
     * @param is_45 ????
     */
    Imu3DM_GX3_25(const char* portname, bool stream_data, bool real_time, bool is_45);
    virtual ~Imu3DM_GX3_25(void);

    bool initialize(uint8_t* message_type, int num_messages);
    bool setStreamThreadParams(char* keyword, int priority, int stacksize, int cpu_id, int delay_ns);
    bool readAccelAngrate(double* accel, double* angrate, double& timestamp);
    bool readStabAccelAngrateMag(double* stab_accel, double* angrate, double* stab_mag, double& timestamp);
    bool readQuat(double* quat, double& timestamp);

  /**
   * @brief Public interface for this specific one.
   */
  public:
    bool openPort(void);
    bool closePort(void);
    bool switchMode(bool);
    bool setCommunicationSettings(void);
    bool setSamplingSettings(void);
    bool initTimestamp(void);
    bool captureGyroBias(void);
    bool setTimeout(double timeout);

    bool startStream(void);
    bool stopStream(void);

    bool readingLoop(void);
    bool stopReadingLoop(void);

    bool receiveAccelAngrate(void);
    bool receiveStabAccelAngrateMag(void);
    bool receiveAccelAngrateOrient(void);
    bool receiveQuat(void);

    bool writeToDevice(int len);
    bool readFromDevice(uint8_t command, int len);
    bool readMisalignedMsgFromDevice(uint8_t cmd, int len);

    void printString(const char* string);
    static inline uint16_t bswap_16(uint16_t x);
    static inline uint32_t bswap_32(uint32_t x);
    static bool isChecksumCorrect(uint8_t* rep, int rep_len);
    void lockData(void);
    void unlockData(void);


  private:

    struct{
      char* keyword_;
      int priority_;
      int stacksize_;
      int cpu_id_;
      int delay_ns_;
    } imu_comm_xeno_info_;

    int fd_;
    ssize_t res_;
    #if __XENO__
      struct rtser_config rt_config_; // RT
     #else
    struct termios config_; // non-RT
    #endif
    fd_set set_;
    struct timespec timeout_;
    uint8_t buffer_[100];

    const char* port_;
    uint8_t* message_type_;
    int num_messages_;
    bool stream_data_;
    bool realtime_;
    bool stop_imu_comm_;
    bool timeout_set_;
    bool debug_timing_;
    bool calc_orient_;
    bool calc_quat_;
    bool is_45_;
    int dec_rate_;

    boost::shared_ptr<boost::thread> imu_comm_thread_;
    rt_mutex mutex_;
    FILE* logfile_rt_;
    FILE* logfile_;
    double delta1_, delta2_, delta3_;
    static const int max_realign_trials_ = 3;

    double accel_[3];
    double stab_accel_[3];
    double angrate_[3];
    double stab_mag_[3];
    double orient_mat_[9];
    double quat_[4];
    double timestamp_;
  };

#endif // IMU_3DM_GX3_25_HPP
