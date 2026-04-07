cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE Release)
project(locomotion_project)

# Establecer el estándar de C++ a C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Flags de optimización para máximo rendimiento matemático
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# Buscar dependencias
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(DART REQUIRED COMPONENTS utils-urdf gui-osg)

# Indicar dónde están los archivos .hpp
include_directories(libs)


# Añadimos kinematics.cpp al ejecutable
add_executable(integracion src/integration_code.cpp)

# Añadimos la carpeta 'libs' para que encuentre robot_config.hpp y kinematics.hpp
target_include_directories(integracion PUBLIC 
    ${DART_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(integracion
    dart
    dart-utils-urdf
    dart-gui-osg
)

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
add_executable(trayectory
    src/generacion_trayectorias.cpp 
    src/kinematics.cpp
    src/memory_setup.cpp
    src/gait_setup.cpp
    src/zmp_control.cpp
)

# Vincular las librerías necesarias
# - Eigen3::Eigen: Para las matemáticas de vectores y matrices
# - Threads::Threads: Para std::thread y std::atomic
# - rt: (Real-Time) Estrictamente necesario para shm_open y memoria compartida en Linux
target_link_libraries(trayectory
    PRIVATE 
    Eigen3::Eigen 
    Threads::Threads 
    rt
)

# Crear el ejecutable combinando los archivos fuente
add_executable(static_test
    src/static_balance.cpp 
    src/kinematics.cpp
    src/memory_setup.cpp
    src/gait_setup.cpp
    src/zmp_control.cpp
)

# Vincular las librerías necesarias
# - Eigen3::Eigen: Para las matemáticas de vectores y matrices
# - Threads::Threads: Para std::thread y std::atomic
# - rt: (Real-Time) Estrictamente necesario para shm_open y memoria compartida en Linux
target_link_libraries(static_test
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
    src/zmp_control.cpp
)

#     src/quadruped_robot.cpp
#     src/gait_generator.cpp
#     src/kinematics_sam.cpp
#     src/bezier_sam.cpp
# )

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