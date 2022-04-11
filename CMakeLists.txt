cmake_minimum_required(VERSION 3.0.2)
project(hero_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_add_env_hooks(99.hero_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

###########
## Build ##
###########

add_executable(battery_manager_laptop src/battery_manager_laptop.cpp)
add_dependencies(battery_manager_laptop ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_manager_laptop
  ${catkin_LIBRARIES}
)
