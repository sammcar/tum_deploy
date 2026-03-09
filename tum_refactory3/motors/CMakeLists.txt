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
# No buscamos fmt aquí con find_package porque DART ya lo trae vinculado

# --- 3. LIBRERÍAS DE HARDWARE (Moteus y Pi3Hat) ---
FetchContent_Declare(
  moteus 
  GIT_REPOSITORY https://github.com/mjbots/moteus.git 
  GIT_TAG f14f789fefeae9caf185b950cf4e32df01d022ea
)
FetchContent_MakeAvailable(moteus)

FetchContent_Declare(
  pi3hat 
  GIT_REPOSITORY https://github.com/mjbots/pi3hat.git 
  GIT_TAG master
)
FetchContent_MakeAvailable(pi3hat)

# --- 4. DIRECTORIOS DE INCLUSIÓN ---
# Se corrigió la sintaxis de los paréntesis aquí
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/../tum_core/include
    ${EIGEN3_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)

# --- 5. EJECUTABLE ---
add_executable(motor_driver 
    src/main.cpp
    ../tum_core/src/kinematics.cpp
)

# --- 6. LINKER (Unión de binarios) ---
target_link_libraries(motor_driver 
    Eigen3::Eigen 
    pthread 
    rt 
    pi3hat 
    moteus::cpp 
    dart         # La librería DART del sistema
    fcl          # Requerido por DART para colisiones
    assimp       # Requerido por DART para modelos 3D
)

# --- 7. OPTIMIZACIONES PI 4 ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    add_compile_options(-O3 -march=armv8-a+crc -mtune=cortex-a72)
    message(STATUS "✅ Optimización activada para Raspberry Pi 4")
endif()
