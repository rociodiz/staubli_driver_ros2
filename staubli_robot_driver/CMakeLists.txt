# Copyright 2025 ICUBE Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Thibault Poignonec (thibault.poignonec@gmail.fr)


cmake_minimum_required(VERSION 3.8)
project(staubli_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the basic ROS 2 build system.
find_package(ament_cmake REQUIRED)

# ROS 2 dependencies used by the driver.
set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  realtime_tools
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Find every ROS 2 dependency listed above.
foreach(DEPENDENCY ${DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Backward ROS provides stack traces when an error occurs.
find_package(backward_ros REQUIRED)

# Boost is used by the networking code through Boost.Asio.
#
# Ubuntu 26.04 uses Boost 1.90. In this version Boost.System is
# header-only, so there is no separate boost_system library/config
# that CMake can find as a COMPONENT.
#
# We therefore find Boost itself and use the header-only target
# Boost::headers below.
find_package(Boost REQUIRED)

# Library sources.
set(LIBRARY_SOURCES
  # Communication protocol and sockets.
  src/communication/protocol.cpp
  src/communication/messages.cpp
  src/communication/socket_factory.cpp
  src/communication/udp_socket.cpp

  # Socket publisher / subscriber.
  src/real_time_socket_subscriber.cpp
  src/real_time_socket_interface.cpp

  # Robot interface.
  src/robot_driver.cpp

  # Mock robot server for testing.
  src/mock_robot_server.cpp
)

# Main library.
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# In newer versions of ament_cmake, ament_target_dependencies()
# has been removed.
#
# We therefore use modern CMake targets directly. These targets
# propagate the include directories, libraries and compile
# definitions required by the ROS 2 dependencies.
#
# Boost::headers is included here because Boost.Asio and
# Boost.System are used as header-only functionality with
# Boost 1.90.
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${control_msgs_TARGETS}
  realtime_tools::realtime_tools
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  Boost::headers
)

# Hardware interface library.
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/staubli_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the ROS 2 dependencies and the main driver library.
#
# We keep all libraries in one target_link_libraries() call using
# the keyword signature so that CMake does not mix the plain and
# keyword forms.
target_link_libraries(${PROJECT_NAME}_hardware_interface PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${control_msgs_TARGETS}
  realtime_tools::realtime_tools
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  ${PROJECT_NAME}
)

# Install headers.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries.
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_hardware_interface
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Python scripts.
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/download_logs.py
  scripts/remove_val3_server.py
  scripts/show_logs.py
  scripts/upload_val3_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files, such as the VAL3 scripts.
install(
  DIRECTORY val3
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip the copyright linter because the source files already
  # contain their own copyright headers.
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint for this package.
  set(ament_cmake_cpplint_FOUND TRUE)

  # ament_lint_auto_find_test_dependencies()

  # Add unit tests.
  find_package(ament_cmake_gtest REQUIRED)

  function(add_unit_test TEST_NAME TEST_SOURCE)
    ament_add_gtest(${TEST_NAME}
      ${TEST_SOURCE}
    )

    target_include_directories(${TEST_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )

    # Keep the plain signature here because ament_add_gtest()
    # creates the test target using the plain CMake signature.
    target_link_libraries(${TEST_NAME}
      ${PROJECT_NAME}
    )
  endfunction()

  add_unit_test(test_primitives_serialization
    test/unit_tests/communication/test_primitives_serialization.cpp)

  add_unit_test(test_message_serialization
    test/unit_tests/communication/test_message_serialization.cpp)

  add_unit_test(test_sockets
    test/unit_tests/communication/test_sockets.cpp)

  add_unit_test(test_rt_socket_interface
    test/unit_tests/communication/test_rt_socket_interface.cpp)

  add_unit_test(test_robot_driver
    test/unit_tests/test_robot_driver.cpp)

  add_unit_test(test_mock_robot_server
    test/unit_tests/test_mock_robot_server.cpp)

  # Add hardware interface test with additional dependencies.
  ament_add_gtest(test_staubli_hardware_interface
    test/unit_tests/test_staubli_hardware_interface.cpp
  )

  target_include_directories(test_staubli_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_staubli_hardware_interface
    ${PROJECT_NAME}
    staubli_robot_driver_hardware_interface
  )
endif()

# Export the main ROS 2 dependencies.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Simple message receiver executable for testing VAL3 communication.
add_executable(test_receive_robot_state
  src/test_receive_robot_state.cpp
)

target_include_directories(test_receive_robot_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(test_receive_robot_state
  ${PROJECT_NAME}
)

# Install the executable.
install(TARGETS test_receive_robot_state
  DESTINATION lib/${PROJECT_NAME}
)

# Export the ros2_control hardware plugin.
pluginlib_export_plugin_description_file(
  hardware_interface
  staubli_hardware_interface.xml
)

ament_package()