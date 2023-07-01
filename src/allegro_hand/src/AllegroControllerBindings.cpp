// AllegroControllerBindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "AllegroController.h"  // Assume the header file name for the AllegroController class

namespace py = pybind11;

PYBIND11_MODULE(AllegroControllerPy, m) {
    py::class_<AllegroController>(m, "AllegroController")
        .def(py::init<>())
        .def("set_joint_positions", &AllegroController::setJointPositions, py::arg("desired_action"), py::arg("absolute")=true)
        .def("set_joint_torques", &AllegroController::setJointTorques)
        .def("get_time_stamp", &AllegroController::getTimeStamp)
        .def("get_joint_positions", &AllegroController::getJointPositions)
        .def("get_joint_positions_and_velocities", &AllegroController::getJointPositionsAndVelocities);
        .def_static("ros_init", &AllegroController::rosInit)
        .def_static("ros_spin_once", &AllegroController::rosSpinOnce);
}
