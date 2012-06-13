cmake_minimum_required(VERSION 2.8)
project(shape_tools)

find_package(catkin REQUIRED)
catkin_stack()

catkin_project(${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  )

include_directories(include)
find_package(shape_msgs REQUIRED)
include_directories(${shape_msgs_INCLUDE_DIRS})

find_package(visualization_msgs REQUIRED)
include_directories(${visualization_msgs_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/shape_to_marker.cpp
  src/shape_extents.cpp)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})
