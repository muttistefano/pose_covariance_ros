cmake_minimum_required(VERSION 3.0.2)
project(pose_covariance_ros)

set(CMAKE_BUILD_TYPE Debug)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  tf2_ros
  tf2_eigen
  rostest
  std_msgs
  moveit_core
  moveit_ros_planning
  message_generation
  # ur_kinematics
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()


add_service_files(
  FILES
  srv_opt.srv
)

add_message_files(
  FILES
  opt_return.msg

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES pose_lie pose_covariance_ros pose_covariance_ros_optimization
#  CATKIN_DEPENDS roscpp
 DEPENDS message_runtime

)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pose_lie
 src/pose_lie.cpp
)

add_library(pose_covariance_ros
 src/pose_covariance_ros.cpp
)

add_library(pose_covariance_ros_optimization
 src/pose_covariance_ros_optimization.cpp
)

target_link_libraries(pose_lie
  ${catkin_LIBRARIES}
  Eigen3::Eigen
 )

target_link_libraries(pose_covariance_ros
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pose_lie
 )



add_dependencies(pose_lie                          ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_covariance_ros               ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



install(TARGETS pose_lie pose_covariance_ros 
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_executable(${PROJECT_NAME}_node              src/pose_covariance_ros_node.cpp)
add_dependencies(${PROJECT_NAME}_node              ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pose_covariance_ros
  pose_lie
 )


#  if(OPT)
#   message("Compiling optimization node")

target_link_libraries(pose_covariance_ros_optimization
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pose_lie
)
add_dependencies(pose_covariance_ros_optimization  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node_optimization src/pose_covariance_ros_node_optimization.cpp)
add_dependencies(${PROJECT_NAME}_node_optimization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node_optimization
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pose_covariance_ros_optimization
  pose_lie
)
# endif()



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/*/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############




if (CATKIN_ENABLE_TESTING)

add_rostest_gtest(test_cpp test/test.launch test/test0.cpp)
add_dependencies(test_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cpp ${catkin_LIBRARIES} ${rostest_LIBRARIES} Eigen3::Eigen pose_covariance_ros pose_lie)
target_compile_options(test_cpp PUBLIC $<$<CONFIG:Release>:-Ofast -march=native -mavx>)


if(ENABLE_COVERAGE_TESTING)
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
  add_code_coverage(
    NAME coverage_report
    DEPENDENCIES test_cpp 
  )
endif()

endif()
