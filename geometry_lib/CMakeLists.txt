cmake_minimum_required(VERSION 3.5)
project(geometry_lib LANGUAGES CXX)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)


# Source files
set(LIBRARY_SOURCES
    src/geometry_lib.cpp  # Adjust the path based on your project structure
)

# Add the executable
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})

# Specify include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)


# Link against the necessary libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  kdl_parser
)

# Create and install the package configuration
include(CMakePackageConfigHelpers)
set(PACKAGE_VERSION "0.1.0")  # Set your package version
configure_package_config_file(
    "geometry_libConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/geometry_libConfig.cmake"
    INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
)

# Install the library
install(TARGETS ${PROJECT_NAME}
    EXPORT geometry_libTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/geometry_libConfig.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}"
)


# Create an executable for testing
add_executable(test_main src/test_main.cpp)
# Link the test executable against the geometry_lib library
target_link_libraries(test_main ${PROJECT_NAME})



# Include the headers
install(DIRECTORY include/
  DESTINATION include
)

# Package information
ament_export_targets(geometry_libTargets)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


