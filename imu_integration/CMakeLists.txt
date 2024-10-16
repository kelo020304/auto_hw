cmake_minimum_required(VERSION 2.8.3)
project(imu_integration)

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Release FORCE )
endif()


#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs )

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(imu_integration
        src/imu_integration_node.cpp
)

target_link_libraries(imu_integration ${catkin_LIBRARIES} )