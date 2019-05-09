#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

class ImuInterface {

  public:

    ImuInterface(const char* portname, bool stream_data, bool realtime, bool is_45);
    virtual ~ImuInterface(void);

    bool initialize(uint8_t* message_type, int num_messages);
    bool setStreamThreadParams(char* keyword, int priority, int stacksize, int cpu_id, int delay_ns);
    bool readAccelAngrate(double* accel, double* angrate, double& timestamp);
    bool readStabAccelAngrateMag(double* stab_accel, double* angrate, double* stab_mag, double& timestamp);
    bool readQuat(double* quat, double& timestamp);
  };

#endif
