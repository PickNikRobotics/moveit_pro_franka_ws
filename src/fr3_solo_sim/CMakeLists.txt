cmake_minimum_required(VERSION 3.22)
project(fr3_solo_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    mjcf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
