cmake_minimum_required(VERSION 3.8)
project(icm20948_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package
ament_python_install_package(${PROJECT_NAME})

# Node entrypoints (simple wrappers)
install(PROGRAMS
  scripts/iio_imu_node
  scripts/qwiic_imu_node
  scripts/dual_mono_sync_check
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
