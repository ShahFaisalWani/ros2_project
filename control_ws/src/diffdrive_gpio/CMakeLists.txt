cmake_minimum_required(VERSION 3.16)
project(diffdrive_gpio LANGUAGES CXX)

# Compiler options for warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Set Windows export behavior
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Define package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find package dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Include directories for header files
include_directories(
  ${PROJECT_SOURCE_DIR}/hardware/include
)


## COMPILE
add_library(
  diffdrive_gpio
  SHARED
  hardware/diffbot_system.cpp
  hardware/wheel.cpp   # Add wheel.cpp here to ensure it's included
)

set(PIGPIO_LIB_PATH "~/pigpio/build")

# Manually set the pigpio libraries
find_library(PIGPIO_LIB pigpio HINTS ${PIGPIO_LIB_PATH})
find_library(PIGPIOD_IF2_LIB pigpiod_if2 HINTS ${PIGPIO_LIB_PATH})

# Link the pigpio libraries to the target
target_link_libraries(diffdrive_gpio PUBLIC
  ${PIGPIO_LIB}
  ${PIGPIOD_IF2_LIB}
)


# Set C++ standard to C++17
target_compile_features(diffdrive_gpio PUBLIC cxx_std_17)

# Include directories for target
target_include_directories(diffdrive_gpio PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/diffdrive_gpio>
)

# Link dependencies and pigpio libraries
ament_target_dependencies(
  diffdrive_gpio PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(diffdrive_gpio PUBLIC
  ${PIGPIO_LIB}
  ${PIGPIOD_IF2_LIB}
)

# Export hardware plugins for ros2_control
pluginlib_export_plugin_description_file(hardware_interface diffdrive_gpio.xml)

## INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_gpio
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/diffdrive_gpio
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffdrive_gpio
)

install(TARGETS diffdrive_gpio
  EXPORT export_diffdrive_gpio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_gpio HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
