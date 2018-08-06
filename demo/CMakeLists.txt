cmake_minimum_required(VERSION 2.8)
project(test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(${PROJECT_NAME}_m  main.cpp)
target_link_libraries(${PROJECT_NAME}_m
                                      ${PCL_LIBRARIES}) 
                                      

target_link_libraries(${PROJECT_NAME}_m LINK_PUBLIC
                                        vlp_driver
                                        ) 