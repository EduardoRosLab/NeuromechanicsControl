cmake_minimum_required(VERSION 2.8.3)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(neuromechanics_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_filters
  gazebo_ros
  dynamic_reconfigure
  baxter_core_msgs
)

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## System dependencies are found with CMake's conventions



#set(BOOST_INCLUDEDIR /usr/include/boost)
#set(BOOST_LIBRARYDIR /usr/lib/x86_64-linux-gnu)
find_package(Boost REQUIRED COMPONENTS system thread)

#if(Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIRS})
#endif()

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

set(WITH_CUDA TRUE)
if(WITH_CUDA)
  find_package(CUDA REQUIRED)
  if (CUDA_FOUND)
    message(STATUS "CUDA FOUND")
    set(CUDA_LIBS cudart)
    set(CUDA_LIB_PATH ${CUDA_TOOLKIT_ROOT_DIR}/lib64)
  endif()
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Analog.msg
  AnalogCompact.msg
  Spike.msg
  Spike_group.msg
  LearningState.msg
  AnalogCompactDelay.msg
  Time.msg
  AnalogCompact_AgonistAntagonist.msg
)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/DynParameters.cfg
    cfg/DynParameters_2Entries.cfg
    cfg/SynchronizerDynParameters.cfg
    cfg/MaeDynParameters.cfg
    cfg/WeightsDynParameters.cfg
    cfg/JointSpringsExample.cfg
    cfg/DelayGeneratorDynParameters.cfg
    cfg/JointsPD.cfg
    cfg/Ekeberg.cfg
    cfg/CocontractionProfileDynParameters.cfg
#   cfg/DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neuromechanics_control
#  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime baxter_core_msgs

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/edlutkernel
  ~/ros_ws/devel/include
  include
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
  ${Boost_LIBRARY_DIRS}
  ${CUDA_LIB_PATH}
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(edlut_ros_interface
  src/ROSTopicInputDriver.cpp src/ROSTopicOutputDriver.cpp
)

add_library(spike_ros_interface
  src/ROSRBFBank_delay.cpp
  src/ROSSpikeDecoderCompact_AgonistAntagonist.cpp
  src/ROSPoissonGenerator.cpp
  src/ROSErrorCalculatorCompact.cpp
  src/ROSTorqueCalculatorAddition.cpp
  src/ROSCocontractionModulation.cpp
)

add_library(baxter_interface
  src/Q_TrajectoryController.cpp
  src/Q_TrajectoryGenerator.cpp
  src/PDSupervisor.cpp
  src/SynchronizerDynParams.cpp
  src/MuscleDynamicsCocontraction.cpp
  src/MuscleDynamicsCocontraction_WithPD.cpp
  src/MuscleDynamicsCocontraction_DirectPD.cpp
  src/MuscleDynamicsCocontraction_WithPD_Ablation.cpp
  src/MuscleDynamicsActiveCocontraction.cpp
)

#add_library(baxter_gazebo_ros_control
#  src/baxter_gazebo_ros_control_plugin.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(edlut_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(edlut_ros_interface neuromechanics_control_generate_messages_cpp)
add_dependencies(spike_ros_interface neuromechanics_control_generate_messages_cpp)
add_dependencies(baxter_interface neuromechanics_control_generate_messages_cpp)
#add_dependencies(edlut_ros_interface ${PROJECT_NAME}_gencfg)

## Declare a C++ executable
add_executable(simulator_node src/simulator_node.cpp)
add_executable(delay_analog_node src/delay_analog_node.cpp)
add_executable(error_estimator_node_compact src/error_estimator_node_compact.cpp)
add_executable(spike_decoder_node_compact_agonist_antagonist src/spike_decoder_node_compact_agonist_antagonist.cpp)
add_executable(poisson_generator_node_delay_compact src/poisson_generator_node_delay_compact.cpp)
add_executable(q_trajectory_generator_node src/q_trajectory_generator_node.cpp)
add_executable(pd_supervisor_node src/pd_supervisor_node.cpp)
add_executable(rbf_node_sync_delay src/rbf_node_sync_delay.cpp)
add_executable(synchronizer_node src/synchronizer_node.cpp)
add_executable(torque_addition_node src/torque_addition_node.cpp)
add_executable(torque_command_remaping src/torque_command_remaping.cpp)
add_executable(median_filter_node_agonist_antagonist src/median_filter_node_agonist_antagonist.cpp)
add_executable(muscle_dynamics src/muscle_dynamics.cpp)
add_executable(muscle_dynamics_withPD src/muscle_dynamics_withPD.cpp)
add_executable(muscle_dynamics_directPD src/muscle_dynamics_directPD.cpp)
add_executable(muscle_dynamics_withPD_ablation src/muscle_dynamics_withPD_ablation.cpp)
add_executable(cocontraction_modulation src/cocontraction_modulation.cpp)
add_executable(muscle_dynamics_active_cocontraction src/muscle_dynamics_active_cocontraction.cpp)

#add_executable(SynchronizerDynParams src/SynchronizerDynParams.cpp)

############################
add_executable(baxter_arm_state_node_sync src/baxter_arm_state_node_sync.cpp) ############################
add_executable(baxter_torque_command_node src/baxter_torque_command_node.cpp) ############################
add_executable(baxter_position_command_node src/baxter_position_command_node.cpp) ########################
############################

target_compile_options(simulator_node PRIVATE ${OpenMP_FLAGS} -fPIC)
target_compile_options(edlut_ros_interface PRIVATE ${OpenMP_FLAGS} -fPIC)
target_compile_options(spike_ros_interface PRIVATE ${OpenMP_FLAGS} -fPIC)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(edlut_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simulator_node neuromechanics_control_generate_messages_cpp)
add_dependencies(delay_analog_node neuromechanics_control_generate_messages_cpp)
add_dependencies(error_estimator_node_compact neuromechanics_control_generate_messages_cpp)
add_dependencies(spike_decoder_node_compact_agonist_antagonist neuromechanics_control_generate_messages_cpp)
add_dependencies(poisson_generator_node_delay_compact neuromechanics_control_generate_messages_cpp)
add_dependencies(q_trajectory_generator_node neuromechanics_control_generate_messages_cpp)
add_dependencies(pd_supervisor_node neuromechanics_control_generate_messages_cpp)
add_dependencies(rbf_node_sync_delay neuromechanics_control_generate_messages_cpp)
add_dependencies(synchronizer_node neuromechanics_control_generate_messages_cpp)
add_dependencies(torque_addition_node neuromechanics_control_generate_messages_cpp)
add_dependencies(torque_command_remaping neuromechanics_control_generate_messages_cpp)
#add_dependencies(SynchronizerDynParams neuromechanics_control_generate_messages_cpp)

add_dependencies(baxter_arm_state_node_sync neuromechanics_control_generate_messages_cpp) ###################################
add_dependencies(baxter_torque_command_node neuromechanics_control_generate_messages_cpp) ###################################
add_dependencies(baxter_position_command_node neuromechanics_control_generate_messages_cpp) ###################################
add_dependencies(median_filter_node_agonist_antagonist neuromechanics_control_generate_messages_cpp)
add_dependencies(muscle_dynamics neuromechanics_control_generate_messages_cpp)
add_dependencies(muscle_dynamics_withPD neuromechanics_control_generate_messages_cpp)
add_dependencies(muscle_dynamics_directPD edlut_ros_osim_generate_messages_cpp)
add_dependencies(muscle_dynamics_withPD_ablation edlut_ros_osim_generate_messages_cpp)
add_dependencies(cocontraction_modulation edlut_ros_osim_generate_messages_cpp)
add_dependencies(muscle_dynamics_active_cocontraction edlut_ros_osim_generate_messages_cpp)



#add_dependencies(step_time_node neuromechanics_control_generate_messages_cpp)
#add_dependencies(baxter_gazebo_ros_control neuromechanics_control_generate_messages_cpp)
#add_dependencies(
#  baxter_gazebo_ros_control
#  ${catkin_EXPORTED_TARGETS}
#)

## Specify libraries to link a library or executable target against
target_link_libraries(edlut_ros_interface
  ${catkin_LIBRARIES}
  edlutkernel
  ${OpenMP_LIBS}
)

target_link_libraries(simulator_node
  ${catkin_LIBRARIES}
  edlutkernel
  edlut_ros_interface
  ${OpenMP_LIBS}
  ${CUDA_LIBS}
)

target_link_libraries(delay_analog_node
  ${catkin_LIBRARIES}
)

target_link_libraries(error_estimator_node_compact
  ${catkin_LIBRARIES}
  spike_ros_interface
  ${Boost_LIBRARIES}
)

target_link_libraries(spike_ros_interface
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
)

target_link_libraries(rbf_node_sync_delay
  ${catkin_LIBRARIES}
  spike_ros_interface
  ${OpenMP_LIBS}
)

target_link_libraries(synchronizer_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  baxter_interface
)

target_link_libraries(spike_decoder_node_compact_agonist_antagonist
  ${catkin_LIBRARIES}
  spike_ros_interface
  ${OpenMP_LIBS}
)

target_link_libraries(poisson_generator_node_delay_compact
  ${catkin_LIBRARIES}
  spike_ros_interface
  ${OpenMP_LIBS}
)

target_link_libraries(baxter_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(q_trajectory_generator_node
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(pd_supervisor_node
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(torque_addition_node
  ${catkin_LIBRARIES}
  spike_ros_interface
  ${Boost_LIBRARIES}
)

target_link_libraries(torque_command_remaping
  ${catkin_LIBRARIES}
  baxter_interface
)

target_link_libraries(baxter_arm_state_node_sync
  ${catkin_LIBRARIES}
)

target_link_libraries(baxter_torque_command_node
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(baxter_position_command_node
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

target_link_libraries(median_filter_node_agonist_antagonist
  ${catkin_LIBRARIES}
)

target_link_libraries(muscle_dynamics
  baxter_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(muscle_dynamics_withPD
  baxter_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(muscle_dynamics_directPD
  baxter_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(muscle_dynamics_withPD_ablation
  baxter_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(cocontraction_modulation
  ${catkin_LIBRARIES}
  spike_ros_interface
  ${Boost_LIBRARIES}
)

target_link_libraries(muscle_dynamics_active_cocontraction
  baxter_interface
  ${catkin_LIBRARIES}
)

#target_link_libraries(SynchronizerDynParams
#  ${catkin_LIBRARIES}
#  baxter_interface
#)

#add_dependencies(
#  baxter_gazebo_ros_control
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(baxter_gazebo_ros_control
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#  gazebo_ros_control
#)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS edlut_ros edlut_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_edlut_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
