cmake_minimum_required(VERSION 2.8.3)
project(ardrone_apps)

find_package(catkin REQUIRED COMPONENTS 
  camera_calibration_parsers
  geometry_msgs
  image_geometry
  rosconsole
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED core)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ardrone_apps
  CATKIN_DEPENDS camera_calibration_parsers geometry_msgs image_geometry rosconsole roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(pixel2xy src/pixel2xy.cpp)
target_link_libraries(pixel2xy ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS pixel2xy RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
