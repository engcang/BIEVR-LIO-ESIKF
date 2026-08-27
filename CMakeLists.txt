cmake_minimum_required(VERSION 3.16)
project(bievr_lio_esikf LANGUAGES C CXX)

### Set compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### Select ROS wrapper
if(DEFINED CATKIN_DEVEL_PREFIX)
    add_subdirectory(ros1)
elseif(DEFINED ENV{AMENT_PREFIX_PATH})
    add_subdirectory(ros2)
endif()
