### Find dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    livox_ros_driver
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    tf2_ros
)
find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

catkin_package()

### Build node
add_executable(${PROJECT_NAME}_mapping src/main_node.cpp)
target_include_directories(${PROJECT_NAME}_mapping PRIVATE
    include
    src
    ../bievr_lio_esikf/include
    ../third_party
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mapping
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARY}
    TBB::tbb
)

### Install
install(TARGETS ${PROJECT_NAME}_mapping
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES rviz.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros1
)
