#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Necesario para convertir std::vector y std::map automáticamente
#include "quadruped_robot.h"
#include "utils_sam.h"

namespace py = pybind11;

// El nombre del módulo en Python será "sam_robot_cpp"
PYBIND11_MODULE(sam_robot_cpp, m) {
    m.doc() = "Modulo de robot cuadrupedo optimizado en C++ con PyBind11";

    // 1. Exponer Vec3
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<double, double, double>(), py::arg("x")=0, py::arg("y")=0, py::arg("z")=0)
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("__repr__", [](const Vec3 &v) {
            return "<Vec3 x=" + std::to_string(v.x) + " y=" + std::to_string(v.y) + " z=" + std::to_string(v.z) + ">";
        });

    // 2. Exponer RobotCommand
    py::class_<RobotCommand>(m, "RobotCommand")
        .def(py::init<>())
        .def_readwrite("vx", &RobotCommand::vx)
        .def_readwrite("vy", &RobotCommand::vy)
        .def_readwrite("wz", &RobotCommand::wz)
        .def_readwrite("body_x", &RobotCommand::body_x)
        .def_readwrite("body_y", &RobotCommand::body_y)
        .def_readwrite("body_z", &RobotCommand::body_z)
        .def_readwrite("roll", &RobotCommand::roll)
        .def_readwrite("pitch", &RobotCommand::pitch)
        .def_readwrite("yaw", &RobotCommand::yaw)
        .def_readwrite("piv_x", &RobotCommand::piv_x)
        .def_readwrite("piv_y", &RobotCommand::piv_y)
        .def_readwrite("leg_modes", &RobotCommand::leg_modes);

    // 3. Exponer QuadrupedRobot
    py::class_<QuadrupedRobot>(m, "QuadrupedRobot")
        .def(py::init<double, double, double, double, double>(), 
             py::arg("body_len"), py::arg("body_wid"), py::arg("l1"), py::arg("l2"), py::arg("l3"))
        .def("step", &QuadrupedRobot::step, 
             py::arg("t"), 
             py::arg("dt"), 
             py::arg("cmd"), 
             py::arg("sensor_data"), 
             py::arg("modo") = "Trote",
             "Ejecuta un paso de simulación y devuelve los ángulos")
        .def("get_default_stance", &QuadrupedRobot::get_default_stance);
}