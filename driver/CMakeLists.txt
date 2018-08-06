cmake_minimum_required(VERSION 2.8)
project(vlp_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package (yaml-cpp REQUIRED)
 
ADD_LIBRARY(${PROJECT_NAME} calibration.cc 
                            cloud_frame.cpp
                            input_dump_file.cpp
                            rawdata.cc )

target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_include_directories (${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

