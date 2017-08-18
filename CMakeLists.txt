cmake_minimum_required(VERSION 2.8.3)
project(qpoases)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
)

##############
## Building ##
##############

include_directories(include ${catkin_INCLUDE_DIRS})

#
# suppress any printline output from qpOASES
#
add_definitions(-D__SUPPRESSANYOUTPUT__)

#
# building qpOASES LIBRARY
#
set(SRCS
  src/BLASReplacement.cpp
  src/Constraints.cpp
  src/Indexlist.cpp
  src/Matrices.cpp
  src/Options.cpp
  src/QProblemB.cpp
  src/SolutionAnalysis.cpp
  src/SubjectTo.cpp
  src/Bounds.cpp
  src/Flipper.cpp
  src/LAPACKReplacement.cpp
  src/MessageHandling.cpp
  src/OQPinterface.cpp 
  src/QProblem.cpp
  src/SQProblem.cpp
  src/Utils.cpp)
add_library(${PROJECT_NAME} ${SRCS})

#
# building example applications
#
set(EXAMPLES
  qrecipe)
 
foreach(EXAMPLE ${EXAMPLES})
  add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
  target_link_libraries(${EXAMPLE} ${PROJECT_NAME})
endforeach(EXAMPLE)


add_executable(vel_example examples/vel_example.cpp)
target_link_libraries(vel_example ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dof2 examples/dof2.cpp)
target_link_libraries(dof2 ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(joint_eef_goal examples/joint_eef_goal.cpp)
target_link_libraries(joint_eef_goal ${PROJECT_NAME} ${catkin_LIBRARIES})

#catkin_package(CATKIN_DEPENDS roscpp)

################
## Installing ##
################

install(TARGETS ${PROJECT_NAME} ${EXAMPLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)
