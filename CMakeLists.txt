cmake_minimum_required(VERSION 3.0.2)
project(keyboard_acq)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)

# Set the path to Spinnaker SDK
set(SPINNAKER_DIR /opt/spinnaker)
set(SPINNAKER_INCLUDE_DIR ${SPINNAKER_DIR}/include)
set(SPINNAKER_LIB_DIR ${SPINNAKER_DIR}/lib)

# Find Spinnaker package
find_library(SPINNAKER_LIBRARIES NAMES Spinnaker PATHS ${SPINNAKER_LIB_DIR})
include_directories(${SPINNAKER_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES keyboard_acq
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SPINNAKER_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(keyboard_publisher src/keyboard_publisher.cpp)
add_executable(camera_node src/camera_node.cpp)

target_link_libraries(keyboard_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(camera_node
  ${catkin_LIBRARIES}
  ${SPINNAKER_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS keyboard_publisher camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

