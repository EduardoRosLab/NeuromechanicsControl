# Install script for directory: /home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/neuromechanics_control/msg" TYPE FILE FILES
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/Analog.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/AnalogCompact.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/Spike.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/Spike_group.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/LearningState.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/AnalogCompactDelay.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/Time.msg"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/msg/AnalogCompact_AgonistAntagonist.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/neuromechanics_control/cmake" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/build/neuromechanics_control/catkin_generated/installspace/neuromechanics_control-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/share/roseus/ros/neuromechanics_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/share/common-lisp/ros/neuromechanics_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/share/gennodejs/ros/neuromechanics_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/lib/python2.7/dist-packages/neuromechanics_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/lib/python2.7/dist-packages/neuromechanics_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/DynParametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/DynParameters_2EntriesConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/SynchronizerDynParametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/MaeDynParametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/WeightsDynParametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/JointSpringsExampleConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/DelayGeneratorDynParametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/JointsPDConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/EkebergConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/include/neuromechanics_control/CocontractionProfileDynParametersConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/lib/python2.7/dist-packages/neuromechanics_control/__init__.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/lib/python2.7/dist-packages/neuromechanics_control/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/neuromechanics_control" TYPE DIRECTORY FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/devel/lib/python2.7/dist-packages/neuromechanics_control/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/build/neuromechanics_control/catkin_generated/installspace/neuromechanics_control.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/neuromechanics_control/cmake" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/build/neuromechanics_control/catkin_generated/installspace/neuromechanics_control-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/neuromechanics_control/cmake" TYPE FILE FILES
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/build/neuromechanics_control/catkin_generated/installspace/neuromechanics_controlConfig.cmake"
    "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/build/neuromechanics_control/catkin_generated/installspace/neuromechanics_controlConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/neuromechanics_control" TYPE FILE FILES "/home/baxter/git_repos/NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/package.xml")
endif()

