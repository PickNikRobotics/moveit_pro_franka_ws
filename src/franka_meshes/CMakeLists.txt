cmake_minimum_required(VERSION 3.22)
project(franka_meshes)

find_package(ament_cmake REQUIRED)

# The whole tree, so package://franka_meshes/meshes/... resolves at the source-tree path.
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}/)
install(FILES NOTICE DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_meshes test/test_meshes.py)
endif()

ament_package()
