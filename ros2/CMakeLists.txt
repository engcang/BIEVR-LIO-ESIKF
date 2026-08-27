### Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

### Build node
add_executable(${PROJECT_NAME}_mapping src/main_node.cpp)
target_include_directories(${PROJECT_NAME}_mapping PRIVATE
    include
    src
    ../bievr_lio_esikf/include
    ../third_party
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_mapping
    geometry_msgs
    livox_ros_driver2
    nav_msgs
    pcl_conversions
    rclcpp
    sensor_msgs
    tf2_ros
)
target_link_libraries(${PROJECT_NAME}_mapping
    ${PCL_COMMON_LIBRARY}
    TBB::tbb
)

### Install
install(TARGETS ${PROJECT_NAME}_mapping
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)
install(FILES rviz.rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
