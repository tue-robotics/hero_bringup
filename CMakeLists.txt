cmake_minimum_required(VERSION 3.5)
project(hero_bringup)

find_package(catkin REQUIRED)

catkin_add_env_hooks(99.hero_functions SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()
