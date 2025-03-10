cmake_minimum_required(VERSION 3.8)
project(faulhaber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

pkg_check_modules(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}
  SHARED
  src/MCDrive.cpp
  src/MsgHandler.cpp
  src/MCUart.cpp
  src/MCNode.cpp
  src/SDOHandler.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

target_link_libraries(${PROJECT_NAME} ${SERIAL_LIBRARIES})

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface ${PROJECT_NAME}_plugin.xml
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
