cmake_minimum_required(VERSION 2.8.3)
project(linear_interpolation_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
  usv_navigation_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(linear_interpolation_planner_node src/linear_interpolation_planner.cpp src/linear_interpolation_planner_node.cpp)
target_link_libraries(linear_interpolation_planner_node ${catkin_LIBRARIES})
add_dependencies(linear_interpolation_planner_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
set(NODE_FILES
  linear_interpolation_planner_node
  )
foreach(nodes ${NODE_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(nodes)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############
