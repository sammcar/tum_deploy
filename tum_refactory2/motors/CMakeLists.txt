cmake_minimum_required(VERSION 3.14)
project(atom51_motors)

# 1. Estándar de C++ (Obligatorio para Moteus y para usar 'inline' en constantes)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# 2. Buscar dependencias base del sistema
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# --- DESCARGA DE LIBRERÍAS DE HARDWARE ---
# FMT (Requerido por Moteus para formatear texto)
FetchContent_Declare(fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 10.1.1)
FetchContent_MakeAvailable(fmt)

# Moteus (¡Aquí está el famoso moteus.h que te faltaba!)
FetchContent_Declare(moteus GIT_REPOSITORY https://github.com/mjbots/moteus.git GIT_TAG f14f789fefeae9caf185b950cf4e32df01d022ea)
FetchContent_MakeAvailable(moteus)

# Pi3Hat (Placa sombrero de la Raspberry Pi para CAN FD)
FetchContent_Declare(pi3hat GIT_REPOSITORY https://github.com/mjbots/pi3hat.git GIT_TAG master)
FetchContent_MakeAvailable(pi3hat)

# 3. Directorios de inclusión (Headers .hpp)
# Aquí unimos tu carpeta libs local con la carpeta global de tum_core
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/../tum_core/include
    ${EIGEN3_INCLUDE_DIRS}
)

# 4. Definición de ejecutables y código fuente (.cpp)
# Incluimos kinematics.cpp porque lo vas a necesitar en el driver para calcular ángulos
add_executable(motor_driver 
    src/main.cpp
    ../tum_core/src/kinematics.cpp
)

# 5. Linkear librerías
# Juntamos las matemáticas (Eigen), tiempo real (rt, pthread) y hardware (moteus, pi3hat)
target_link_libraries(motor_driver 
    Eigen3::Eigen 
    pthread 
    rt 
    pi3hat 
    moteus::cpp 
    fmt::fmt
)

# 6. Optimizaciones agresivas para Raspberry Pi 4
# Esto hará que el bucle de 5ms corra aún más rápido
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    add_compile_options(-O3 -march=armv8-a+crc -mtune=cortex-a72)
    message(STATUS "✅ Optimización activada para Raspberry Pi 4 (Cortex-A72)")
endif()
