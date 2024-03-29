/*
copyright stuff
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

#include <imu-core/imu_interface.hpp>
#include <imu-core/imu_3DM_GX3_45.hpp>

namespace py = pybind11;

// lets "i"_a be the same as py::arg("i") for keyword arguments
using namespace pybind11::literals;

using namespace imu_core;
using namespace imu_core::imu_3DM_GX3_45;

// have to do this because initialize() is a pure virtual method
class PyImuInterface : public ImuInterface{
public:
  using ImuInterface::ImuInterface;

  bool initialize() override { 
    // deprecated in newer versions of pybind11, just rename PYBIND11_OVERLOAD_PURE --> PYBIND11_OVERRIDE_PURE 
    // without changing anything else
    PYBIND11_OVERLOAD_PURE( 
      bool, // return type
      ImuInterface, // parent class
      initialize, // name of function
      // no arguments
    );
  }
};


PYBIND11_MODULE(imu_core_cpp, m){
  py::class_<ImuMsg>(m, "ImuMsg")
    .def(py::init<>())
    .def("is_valid", &ImuMsg::is_valid)
    .def("reply_debug_string", &ImuMsg::reply_debug_string)
    .def("command_debug_string", &ImuMsg::command_debug_string);

  py::class_<ImuInterface, PyImuInterface /* trampoline class */>(m, "ImuInterface")
    .def(py::init<const std::string>())
    .def("initialize", &ImuInterface::initialize)
    .def("get_acceleration", &ImuInterface::get_acceleration)
    .def("get_angular_rate", &ImuInterface::get_angular_rate)
    .def("get_quaternion_xyzw", &ImuInterface::get_quaternion_xyzw)
    .def("get_quaternion_wxyz", &ImuInterface::get_quaternion_wxyz)
    .def_static("bswap_16", &ImuInterface::bswap_16, "x"_a)
    .def_static("bswap_32", &ImuInterface::bswap_32, "x"_a);

  py::class_<Imu3DM_GX3_45, ImuInterface>(m, "Imu3DM_GX3_45")
    .def(py::init<const std::string , const bool&>(), "port_name"_a, "stream_data"_a=false)
    .def("initialize", &Imu3DM_GX3_45::initialize)
    .def("open_usb_port", &Imu3DM_GX3_45::open_usb_port, "bauderate"_a)
    .def("receive_message", &Imu3DM_GX3_45::receive_message, "msg"_a, "stream_mode"_a)
    .def("is_checksum_correct", &Imu3DM_GX3_45::is_checksum_correct, "msg"_a)
    .def("read_misaliged_msg_from_device", &Imu3DM_GX3_45::read_misaligned_msg_from_device, "msg"_a)
    .def("send_message", &Imu3DM_GX3_45::send_message, "msg"_a)
    .def("start_streaming_data", &Imu3DM_GX3_45::start_streaming_data)
    .def_static("reading_loop_helper", &Imu3DM_GX3_45::reading_loop_helper, "object"_a)
    .def("reading_loop", &Imu3DM_GX3_45::reading_loop)
    .def("stop_reading_loop", &Imu3DM_GX3_45::stop_reading_loop);
}
