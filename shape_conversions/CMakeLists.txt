project(${CATKIN_CURRENT_STACK}_conversions)

catkin_project(${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  )

include_directories(include)

find_package(shape_msgs REQUIRED)
include_directories(${shape_msgs_INCLUDE_DIRS})

find_package(visualization_msgs REQUIRED)
include_directories(${visualization_msgs_INCLUDE_DIRS})

# horrible hack: catkin does not find this, although it seems it should
include_directories(/opt/ros/fuerte/include/)

add_library(${PROJECT_NAME} src/shape_to_marker.cpp)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include/${CATKIN_CURRENT_STACK}
        FILES_MATCHING PATTERN "*.h")
