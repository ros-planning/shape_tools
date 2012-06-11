cmake_minimum_required(VERSION 2.8)
project(shape_tools)

find_package(catkin REQUIRED)
catkin_stack()

add_subdirectory(shape_utils)
add_subdirectory(shape_conversions)

install(FILES stack.xml
        DESTINATION share/${CATKIN_CURRENT_STACK})

