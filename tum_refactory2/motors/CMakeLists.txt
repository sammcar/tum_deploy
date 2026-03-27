cmake_minimum_required(VERSION 3.14)
project(atom51_motors)


# --- 1. ESTÁNDAR DE C++ ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- NUEVO: ACTIVACIÓN AUTOMÁTICA DE CCACHE ---
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "✅ ccache encontrado y activado para compilar a máxima velocidad")
else()
    message(WARNING "❌ ccache no encontrado. Instálalo con: sudo apt install ccache")
endif()
# ------------

include(FetchContent)

# --- 2. DEPENDENCIAS DEL SISTEMA ---
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(DART REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) 

# --- 3. DEPENDENCIAS EXTERNAS (Moteus y Pi3Hat) ---
set(FETCHCONTENT_QUIET OFF) 
FetchContent_Declare(
  moteus 
  GIT_REPOSITORY https://github.com/mjbots/moteus.git 
  GIT_TAG f14f789fefeae9caf185b950cf4e32df01d022ea
  GIT_PROGRESS TRUE   
)
FetchContent_MakeAvailable(moteus)

FetchContent_Declare(
  pi3hat 
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git 
  GIT_TAG master
  GIT_PROGRESS TRUE   
)
FetchContent_MakeAvailable(pi3hat)

# --- 4. DIRECTORIOS DE INCLUSIÓN (Tus .hpp) ---
# Aquí le decimos a CMake dónde buscar cuando hagas #include "archivo.hpp"
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/../tum_core/include
    ${EIGEN3_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS} 
)

# --- 5. EJECUTABLES (Tus .cpp) ---

# A) El Cerebro Principal (Añadimos los nuevos .cpp modulares)
add_executable(motor_driver 
    src/main.cpp
    src/utils_motors.cpp      # <--- NUEVO: La lógica de hardware
    src/control_dart.cpp      # <--- NUEVO: La matemática física
    src/gait_planner.cpp      # <--- NUEVO: Generación de trayectoria
    src/parameters_robot.cpp      # <--- NUEVO: Constantes y configuración
    ../tum_core/src/kinematics.cpp
)

# B) Test de IMU Independiente (Ya que imu.cpp tiene su propio main)
add_executable(test_imu 
    src/imu.cpp
)

# C) Test Asíncrono (Si aún lo conservas en tu carpeta)
# Si ya borraste test_new_motors.cpp, puedes eliminar este bloque.
# add_executable(test_new_motors src/test_new_motors.cpp)

# --- 6. LINKER (Unión de librerías a tus ejecutables) ---

# Linker para el programa principal
target_link_libraries(motor_driver 
    Boost::system     # <--- Vital para boost::asio en el main
    Eigen3::Eigen 
    pthread 
    rt 
    pi3hat 
    moteus::cpp 
    dart         
    fcl          
    assimp       
)

# Linker para el test del IMU (Solo necesita hilos básicos)
target_link_libraries(test_imu 
    pthread 
)

# Linker para test_new_motors (Descomenta si usas el ejecutable C)
# target_link_libraries(test_new_motors Boost::system Eigen3::Eigen pthread rt pi3hat moteus::cpp dart fcl assimp)


# --- 7. OPTIMIZACIONES PI 4 ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    add_compile_options(-O3 -march=armv8-a+crc -mtune=cortex-a72)
    message(STATUS "✅ Optimización activada para Raspberry Pi 4")
endif()
