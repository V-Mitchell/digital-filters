cmake_minimum_required(VERSION 3.6.1)
project(kalman_filter)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

add_executable(${PROJECT_NAME} main.cpp)