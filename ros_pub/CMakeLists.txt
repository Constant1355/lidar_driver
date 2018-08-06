cmake_minimum_required(VERSION 3.5)

PROJECT(ros_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
include_directories(${catkin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

ADD_EXECUTABLE(${PROJECT_NAME}_m  main.cpp)
target_link_libraries(${PROJECT_NAME}_m
                                    ${PCL_LIBRARIES} 
                                    ${catkin_LIBRARIES}
                                    vlp_driver)

