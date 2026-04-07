cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE Release)
project(CuadrupedoDartSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(DART REQUIRED COMPONENTS utils-urdf gui-osg)

# Añadimos kinematics.cpp al ejecutable
add_executable(simulacion src/main.cpp src/kinematics.cpp)

# Añadimos la carpeta 'libs' para que encuentre robot_config.hpp y kinematics.hpp
target_include_directories(simulacion PUBLIC 
    ${DART_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(simulacion
    dart
    dart-utils-urdf
    dart-gui-osg
)