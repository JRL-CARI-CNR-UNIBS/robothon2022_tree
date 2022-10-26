cmake_minimum_required(VERSION 3.0.2)
project(robothon2022_tree)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  behaviortree_cpp_v3
  skills_executer
  skills_executer_msgs
  manipulation_msgs
  actionlib_msgs
  actionlib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
 roscpp
 std_msgs
 behaviortree_cpp_v3
 skills_executer
 skills_executer_msgs
 manipulation_msgs
 actionlib_msgs
 actionlib
 DEPENDS
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/bt_goto_class.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

add_executable(        run_tree src/run_tree.cpp)
add_dependencies(      run_tree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( run_tree
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
add_executable(        run_pc_ewaste src/run_pc_ewaste.cpp)
add_dependencies(      run_pc_ewaste ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( run_pc_ewaste
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
