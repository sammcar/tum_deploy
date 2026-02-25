cmake_minimum_required(VERSION 3.14)
project(motors)

# 1. Estándar de C++ (Moteus requiere C++17 o superior)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 2. Buscar dependencias
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# FMT
FetchContent_Declare(fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 10.1.1)
FetchContent_MakeAvailable(fmt)

# Moteus
FetchContent_Declare(moteus GIT_REPOSITORY https://github.com/mjbots/moteus.git GIT_TAG f14f789fefeae9caf185b950cf4e32df01d022ea)
FetchContent_MakeAvailable(moteus)

# Pi3Hat
FetchContent_Declare(pi3hat GIT_REPOSITORY https://github.com/mjbots/pi3hat.git GIT_TAG master)
FetchContent_MakeAvailable(pi3hat)

# 3. Directorios de inclusión
# Asumimos que tus .hpp están en la carpeta 'include'
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

# 4. Definición de ejecutables

# --- Ejecutable Principal (Control de Motores) ---
add_executable(rex_control 
    src/main.cpp
    # Si tienes archivos .cpp adicionales para kinematics o setup, agrégalos aquí:
    # src/setup.cpp 
)

target_link_libraries(rex_control Eigen3::Eigen pthread rt pi3hat moteus::cpp fmt::fmt)


# 6. Optimizaciones para Raspberry Pi 4 (Opcional pero recomendado)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    add_compile_options(-O3 -march=armv8-a+crc -mtune=cortex-a72)
    message(STATUS "Optimización activada para Raspberry Pi 4 (Cortex-A72)")
endif()
