cmake_minimum_required(VERSION 3.14)
project(rex_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ==============================================================================
# 1. DEPENDENCIAS EXTERNAS (Copiadas de tu configuración original)
# ==============================================================================

# Eigen3 (Matemáticas)
find_package(Eigen3 REQUIRED)

# FMT (Formateo de texto)
FetchContent_Declare(fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 10.1.1)
FetchContent_MakeAvailable(fmt)

# Moteus (Controladores)
FetchContent_Declare(moteus GIT_REPOSITORY https://github.com/mjbots/moteus.git GIT_TAG f14f789fefeae9caf185b950cf4e32df01d022ea)
FetchContent_MakeAvailable(moteus)

# Pi3Hat (Comunicación CAN-FD para Raspberry Pi)
FetchContent_Declare(pi3hat GIT_REPOSITORY https://github.com/mjbots/pi3hat.git GIT_TAG master)
FetchContent_MakeAvailable(pi3hat)

# ==============================================================================
# 2. CONFIGURACIÓN DEL PROYECTO
# ==============================================================================

# Incluir el directorio actual (.) para que encuentre tus .hpp
# (robot_config.hpp, shared_memory.hpp, utils.hpp, telemetry.hpp, kinematics.hpp)
include_directories(.)

# Si tienes archivos .cpp adicionales (no headers), agrégalos aquí.
# Dado que tu código actual parece ser 'header-only' (funciones inline en .hpp),
# solo necesitamos compilar el main.cpp.

# ==============================================================================
# 3. EJECUTABLES
# ==============================================================================

add_executable(rex_main main.cpp)

# Enlazado de librerías
target_link_libraries(rex_main
    PUBLIC
    moteus::cpp      # Librería C++ de Moteus
    fmt::fmt         # Librería de formateo
    pi3hat           # Librería del hat (si usas rpi)
    rt               # Real-time extensions (necesario para timers precisos)
    pthread          # POSIX Threads (necesario para std::thread y atomic)
    Eigen3::Eigen    # Si usas álgebra matricial en el futuro
)
