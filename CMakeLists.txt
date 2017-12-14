cmake_minimum_required(VERSION 2.8.3)
project(p_rob_interface)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  myp_ros
  roscpp
)

catkin_python_setup()

include_directories(include ${catkin_INCLUDE_DIRS})

set(_SOURCES
    src/p_rob_interface_commands.cpp
    src/p_rob_interface_state.cpp
    src/p_rob_interface_trajectory.cpp
    )

catkin_package(CATKIN_DEPENDS
               DEPENDS moveit_msgs myp_ros roscpp
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})    

add_library(${PROJECT_NAME} ${_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
