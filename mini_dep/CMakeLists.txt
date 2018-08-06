cmake_minimum_required(VERSION 2.8)
project(min_dep)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package (yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
                        vlp_driver
                        displayer
                        yaml-cpp
                        -pthread)

