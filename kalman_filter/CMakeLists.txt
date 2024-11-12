cmake_minimum_required(VERSION 3.6.1)
project(kalman_filter)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -O3")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

include_directories(/usr/include/eigen3/Eigen)

add_executable(
    ${PROJECT_NAME}
    main.cpp
    src/KalmanFilter.cpp
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)