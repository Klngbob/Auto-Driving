cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation
  std_srvs
)

add_service_files(
  FILES
  saveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(ALL_TARGET_LIBRARIES "")
include(cmake/eigen.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/geographic.cmake)
include(cmake/global_definition.cmake)
include(cmake/YAML.cmake)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})