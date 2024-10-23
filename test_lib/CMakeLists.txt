cmake_minimum_required(VERSION 3.5)
project(test_lib LANGUAGES CXX)

# Imposta CMAKE_PREFIX_PATH per trovare la libreria geometry_lib
set(CMAKE_PREFIX_PATH "/path/to/geometry_lib/install")  # Sostituisci con il percorso della tua installazione

# Find the geometry_lib package
find_package(geometry_lib REQUIRED)  # Ensure this matches the package name defined in geometry_lib's CMakeLists.txt
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)

# Include directories (if you add any headers in the include directory in the future)
include_directories(include)

# Source files
set(SOURCES
    src/test_main2.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})
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

# Link against the geometry_lib
target_link_libraries(${PROJECT_NAME} geometry_lib)

# Install rules (optional)
install(TARGETS ${PROJECT_NAME}
    DESTINATION bin
)
