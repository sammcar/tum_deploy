cmake_minimum_required(VERSION 3.14)
project(sam_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# --- 1. FMT (NECESARIA porque la usas abajo en fmt::fmt) ---
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 10.1.1 
)
FetchContent_MakeAvailable(fmt)

# --- 2. Moteus ---
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        f14f789fefeae9caf185b950cf4e32df01d022ea
)
FetchContent_MakeAvailable(moteus)

# --- 3. Pi3Hat (RAMA MASTER) ---
FetchContent_Declare(
  pi3hat
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git
  GIT_TAG master
)
FetchContent_MakeAvailable(pi3hat)

# --- LIBRERÍA INTERNA ---
include_directories(libs)

add_library(sam_libs
    libs/bezier_sam.cpp
    libs/gait_generator.cpp
    libs/kinematics_sam.cpp
    libs/quadruped_robot.cpp
)
target_link_libraries(sam_libs moteus::cpp fmt::fmt)

# --- EJECUTABLES ---

# Controller
add_executable(controller robot_controller.cpp)
target_link_libraries(controller rt pi3hat moteus::cpp fmt::fmt) 

# Leg Test
add_executable(leg_test single_leg.cpp)
target_link_libraries(leg_test 
    sam_libs 
    moteus::cpp 
    pi3hat       # CRÍTICO
    fmt::fmt
)
