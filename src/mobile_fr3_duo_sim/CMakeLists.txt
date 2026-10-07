cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_sim)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config description launch maps mjcf objectives params DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS script/base_twist_to_planar.py script/lidar_flattener.py
                script/odom_twist_to_base.py script/scan_merger.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
