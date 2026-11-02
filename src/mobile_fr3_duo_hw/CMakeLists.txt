cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    params
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

install(PROGRAMS script/base_odometry_bridge.py script/spine_joint_state_publisher.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_base_odometry_bridge test/test_base_odometry_bridge.py)
endif()

ament_package()
