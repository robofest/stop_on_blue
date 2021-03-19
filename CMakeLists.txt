cmake_minimum_required(VERSION 3.0.2)
project(stop_on_blue)

# YOUR CODE HERE
# Fill in (1)-(6) below

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

  # (1) Add image processing and dynamic reconfigure packages
  # ...
  cv_bridge
  image_transport
  sensor_msgs
  dynamic_reconfigure
)

# (2) Add OpenCV as required package
# find_package( ... )
find_package(OpenCV REQUIRED)

# (3) Generate Dynamic Reconfigure Files
# generate_dynamic_reconfigure_options( ... )
generate_dynamic_reconfigure_options(cfg/StopOnBlue.cfg)

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # (4) Include OpenCV Headers
  # ...

  include
)

add_executable(stop_on_blue src/stop_on_blue.cpp)
# (5) Add Dynamic reconfigure dependencies
# add_dependencies( ... )
add_dependencies(stop_on_blue ${PROJECT_NAME}_gencfg)
target_link_libraries(stop_on_blue
    ${catkin_LIBRARIES}
    # (6) Link OpenCV Libraries
    ${OpenCV_LIBS}
)
