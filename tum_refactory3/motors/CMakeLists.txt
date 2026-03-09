cmake_minimum_required(VERSION 3.14)
project(atom51_motors)

# 1. Estándar de C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 2. Buscar dependencias del sistema
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(DART REQUIRED)
find_package(Boost REQUIRED COMPONENTS system) # <--- NUEVO: Buscar librería Boost

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

FetchContent_Declare(
  sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG ef9551ff429899b5adae66eabd5a23f165953199
  GIT_PROGRESS TRUE   
)

FetchContent_GetProperties(sophus)
if(NOT sophus_POPULATED)
  FetchContent_Populate(sophus)
endif()

# --- 4. DIRECTORIOS DE INCLUSIÓN ---
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/../tum_core/include
    ${EIGEN3_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${sophus_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS} # <--- NUEVO: Incluir cabeceras de Boost
)

# --- 5. EJECUTABLES ---
# Tu ejecutable original
add_executable(motor_driver 
    src/main.cpp
    ../tum_core/src/kinematics.cpp
)

# NUEVO: El ejecutable para tu test asíncrono
add_executable(test_new_motors 
    src/test_new_motors.cpp
)

# --- 6. LINKER (Unión de binarios) ---
# Linker original
target_link_libraries(motor_driver 
    Eigen3::Eigen 
    pthread 
    rt 
    pi3hat 
    moteus::cpp 
    dart         
    fcl          
    assimp       
)

# NUEVO: Linker para tu test asíncrono (Incluye Boost::system)
target_link_libraries(test_new_motors 
    Boost::system     # <--- Vital para usar boost::asio
    Eigen3::Eigen 
    pthread 
    rt 
    pi3hat 
    moteus::cpp 
    dart         
    fcl          
    assimp       
)

# --- 7. OPTIMIZACIONES PI 4 ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    add_compile_options(-O3 -march=armv8-a+crc -mtune=cortex-a72)
    message(STATUS "✅ Optimización activada para Raspberry Pi 4")
endif()
