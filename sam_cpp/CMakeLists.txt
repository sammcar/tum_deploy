cmake_minimum_required(VERSION 3.14)
project(sam_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DEPENDENCIAS EXTERNAS ---

include(FetchContent)

# 1. FMT (Para logs y strings modernos)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 10.1.1 
)
FetchContent_MakeAvailable(fmt)

# 2. Moteus (Controladores de motores)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG main 
)
FetchContent_MakeAvailable(moteus)

# --- LIBRERÍA INTERNA (Tus algoritmos) ---

include_directories(libs)

# Creamos una librería llamada "sam_libs" con todo lo que hay en la carpeta libs
add_library(sam_libs
    libs/bezier_sam.cpp
    libs/gait_generator.cpp
    libs/kinematics_sam.cpp
    libs/quadruped_robot.cpp
    # Nota: bindings.cpp parece ser para Python, si es C++ puro no lo incluyas aquí a menos que lo uses.
)

# Vinculamos las dependencias que tus librerías necesitan
target_link_libraries(sam_libs moteus::cpp fmt::fmt)


# --- EJECUTABLES ---

# 1. El Controlador (Joystick + Shared Memory)
add_executable(controller robot_controller.cpp)
# IMPORTANTE: 'rt' es necesario para shm_open (Shared Memory)
target_link_libraries(controller rt) 

# 2. Test de Visualización de Patas
#add_executable(leg_test leg_visualizer.cpp)
# Vinculamos con tu librería interna 'sam_libs'
#target_link_libraries(leg_test sam_libs moteus::cpp fmt::fmt)

# 3. Position (Parece ser otro test o control de posición)
#add_executable(position position.cpp)
#target_link_libraries(position sam_libs moteus::cpp fmt::fmt)
