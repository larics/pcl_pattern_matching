cmake_minimum_required(VERSION 2.8.3)
project(pcl_pattern_matching)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             sensor_msgs
             tf
             dynamic_reconfigure
             roslib
             cv_bridge
             image_transport
             mavros)

generate_dynamic_reconfigure_options(config/PatternMatchingParameters.cfg)

list(INSERT CMAKE_MODULE_PATH 0 "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
add_compile_options(-std=c++14)

catkin_package(INCLUDE_DIRS LIBRARIES DEPENDS PCL OpenCV GeographicLib)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_executable(pattern_matching_node src/PatternMatchingNode.cpp)
target_link_libraries(pattern_matching_node ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(pattern_matching_node ${catkin_EXPORTED_TARGETS})
