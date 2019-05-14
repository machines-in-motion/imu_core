#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP


// /**
//  * @brief This class allow the user to write or read some msg.
//  */
// class ImuMsg
// {
// public:
//   /**
//    * @brief Construct a new ImuMsg object.
//    */
//   ImuMsg()
//   {
//     command_.clear();
//     reply_.clear();
//   }
//   /**
//    * @brief Destroy the ImuMsg object.
//    */
//   ~ImuMsg(){}
//   /**
//    * @brief Check if the command_ and the reply_ objects has been initialized.
//    */
//   bool is_valid()
//   {
//     return (command_.size() > 0) && (reply_.size() > 0);
//   }
//   /**
//    * @brief Create a debug string for the command
//    * 
//    * @return std::string the debug string
//    */
//   std::string command_debug_string() const
//   {
//     std::ostringstream cmd_debug_string;
//     cmd_debug_string << "[ ";
//     for (unsigned i=0 ; i<command_.size() ; ++i)
//     {
//       cmd_debug_string << std::hex << std::setfill('0') << std::setw(2)
//                       << std::uppercase << (command_[i] & 0xFF) << " ";
//     }
//     cmd_debug_string << "]";
//     return cmd_debug_string.str();
//   }
//   /**
//    * @brief Create a debug string for the reply
//    * 
//    * @return std::string the debug string
//    */
//   std::string reply_debug_string() const
//   {
//     std::ostringstream cmd_debug_string;
//     cmd_debug_string << "[ ";
//     for (unsigned i=0 ; i<reply_.size() ; ++i)
//     {
//       cmd_debug_string << std::hex << std::setfill('0') << std::setw(2)
//                       << std::uppercase << (reply_[i] & 0xFF) << " ";
//     }
//     cmd_debug_string << "]";
//     return cmd_debug_string.str();
//   }
// public:
//   /**
//    * @brief Contain the command to be written or that has been written
//    */
//   std::vector<uint8_t> msg;
//   /**
//    * @brief contain the device reply
//    */
//   std::vector<uint8_t> reply_;
// };

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
