cmake_minimum_required(VERSION 3.20)
project(dji_motor_hardwear_interface)

# 设置C++版本 -std=c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Debug
option(DEBUG "启动debug发布者，发布电机信息" ON)

add_subdirectory(../../submodule_lib/Candriver ${CMAKE_CURRENT_BINARY_DIR}/Candriver_build)

message("Candriver dir:" ${CMAKE_CURRENT_LIST_DIR}/../../submodule_lib/Candriver)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

if(DEBUG)
  find_package(rm_interface REQUIRED)
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")

message(${SOURCES})

add_library(dji_motor_hardwear_interface SHARED ${SOURCES})



if(DEBUG)
    target_compile_definitions(dji_motor_hardwear_interface PRIVATE DEBUG)
    ament_target_dependencies(dji_motor_hardwear_interface rm_interface)
    message("Debug mode is ON")
else()
    message("Debug mode is OFF")
endif()

ament_target_dependencies(dji_motor_hardwear_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  serial_driver
  realtime_tools
)

target_link_libraries(dji_motor_hardwear_interface
  candriver
)

target_include_directories(dji_motor_hardwear_interface
  PUBLIC
  include
)

ament_export_include_directories(
  include
)

pluginlib_export_plugin_description_file(hardware_interface dji_motor_hardwear_interface.xml)

install(FILES
  dji_motor_hardwear_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS dji_motor_hardwear_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(dji_motor_hardwear_interface)

# test part
install(
  DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
