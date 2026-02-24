cmake_minimum_required(VERSION 3.14)
project(sam_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# --- Dependencias Externas ---
find_package(Eigen3 REQUIRED)

# FMT
FetchContent_Declare(fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 10.1.1)
FetchContent_MakeAvailable(fmt)

# Moteus
FetchContent_Declare(moteus GIT_REPOSITORY https://github.com/mjbots/moteus.git GIT_TAG f14f789fefeae9caf185b950cf4e32df01d022ea)
FetchContent_MakeAvailable(moteus)

# Pi3Hat
FetchContent_Declare(pi3hat GIT_REPOSITORY https://github.com/mjbots/pi3hat.git GIT_TAG master)
FetchContent_MakeAvailable(pi3hat)

# --- Librerías Internas ---
include_directories(libs)
add_library(sam_libs
    libs/bezier_sam.cpp
    libs/gait_generator.cpp
    libs/kinematics_sam.cpp
    libs/quadruped_robot.cpp
)
target_link_libraries(sam_libs moteus::cpp fmt::fmt)

# --- Ejecutables Existentes ---
add_executable(controller robot_controller.cpp)
target_link_libraries(controller rt pi3hat moteus::cpp fmt::fmt) 

add_executable(leg_test single_leg.cpp)
target_link_libraries(leg_test sam_libs moteus::cpp pi3hat fmt::fmt)

add_executable(test pruebas.cpp)
target_link_libraries(test sam_libs moteus::cpp pi3hat fmt::fmt)

add_executable(torque pruebas_torque.cpp)
target_link_libraries(torque sam_libs moteus::cpp pi3hat fmt::fmt)

add_executable(move move_legs.cpp)
target_link_libraries(move sam_libs moteus::cpp pi3hat fmt::fmt)

add_executable(udp conexion_udp.cpp)
target_link_libraries(udp sam_libs moteus::cpp pi3hat fmt::fmt)

# --- NUEVO: Generador de Marcha ---
add_executable(walk_v1 walk_v1.cpp)
target_link_libraries(walk_v1 
    Eigen3::Eigen
    rt 
    pthread
)

add_executable(gait_v1 walk_sam.cpp)

target_link_libraries(gait_v1
    Eigen3::Eigen
    rt
    pthread
)

add_executable(walk_v2 walk_v2.cpp)
target_link_libraries(walk_v2
    Eigen3::Eigen
    rt
    pthread
)

add_executable(gait_v2 walk_sam_v2.cpp)

target_link_libraries(gait_v2
    Eigen3::Eigen
    rt
    pthread
)

add_executable(walk_v3 walk_v3.cpp)
target_link_libraries(walk_v3
    Eigen3::Eigen
    rt
    pthread
)

add_executable(walk_v4 walk_v4.cpp)
target_link_libraries(walk_v4
    Eigen3::Eigen
    rt
    pthread
)

add_executable(walk_v5 walk_v5.cpp)
target_link_libraries(walk_v5
    Eigen3::Eigen
    rt
    pthread
)

add_executable(cleaner memory_cleaner.cpp)
target_link_libraries(cleaner
    Eigen3::Eigen
    rt
    pthread
)

add_executable(single_torque torque_single_leg.cpp)
target_link_libraries(single_torque
    Eigen3::Eigen
    rt
    pthread
)
