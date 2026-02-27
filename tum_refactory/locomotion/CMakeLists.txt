cmake_minimum_required(VERSION 3.10)
project(locomotion_project)

# Establecer el estándar de C++ a C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Flags de optimización para máximo rendimiento matemático
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# Buscar dependencias
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Indicar dónde están los archivos .hpp
include_directories(libs)

# Crear el ejecutable combinando los archivos fuente
add_executable(walking_sam 
    src/main.cpp 
    src/kinematics.cpp
    src/memory_setup.cpp
    src/gait_setup.cpp
    src/zmp_control.cpp
)

# Vincular las librerías necesarias
# - Eigen3::Eigen: Para las matemáticas de vectores y matrices
# - Threads::Threads: Para std::thread y std::atomic
# - rt: (Real-Time) Estrictamente necesario para shm_open y memoria compartida en Linux
target_link_libraries(walking_sam 
    PRIVATE 
    Eigen3::Eigen 
    Threads::Threads 
    rt
)

# Crear el ejecutable combinando los archivos fuente
add_executable(test 
    src/test.cpp 
    src/kinematics.cpp
    src/memory_setup.cpp
    src/gait_setup.cpp
)

# Vincular las librerías necesarias
# - Eigen3::Eigen: Para las matemáticas de vectores y matrices
# - Threads::Threads: Para std::thread y std::atomic
# - rt: (Real-Time) Estrictamente necesario para shm_open y memoria compartida en Linux
target_link_libraries(test 
    PRIVATE 
    Eigen3::Eigen 
    Threads::Threads 
    rt
)