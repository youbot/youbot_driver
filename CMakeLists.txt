PROJECT(YouBot_Driver)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

OPTION(USE_ROS "Enable ROS as compile tool" OFF)

########### ROS ###########
SET(ROS_ROOT_PATH $ENV{ROS_ROOT})
IF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  ADD_DEFINITIONS(-DUSE_ROS_LOGGING)
ENDIF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)

########### cmake configuration and compiler flags ###########
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_CXX_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_C_FLAGS_RELEASE "-fPIC -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-fPIC -Wall")
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #static libraries
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) #executables
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) #dynamic libraries (plugins)

# find required and optional dependencies

########### Boost ###########
#SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)
IF (Boost_FOUND)
  LINK_LIBRARIES(boost_thread boost_date_time boost_filesystem boost_system boost_regex)
  MESSAGE(STATUS "SUCCESSFUL: Boost found")
ELSE (Boost_FOUND)
  MESSAGE(STATUS "ERROR: Boost not found")
ENDIF (Boost_FOUND)

########### Doxygen ###########
SET(DOXYFILE_LATEX "NO")
SET(DOXYFILE_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/doc)
include(UseDoxygen)

########### Simple Open EtherCAT Master Library ###########
add_subdirectory(soem)

INCLUDE_DIRECTORIES(
 ${PROJECT_SOURCE_DIR}
 ${PROJECT_SOURCE_DIR}/soem/src
)

########### CppUnit and tests ###########
FIND_PACKAGE(CppUnit)
IF(CppUnit_FOUND)
	MESSAGE(STATUS "SUCCESSFUL: CPP_UNIT found")
  add_subdirectory(testing)
#ELSE(CppUnit_FOUND)
#	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
ENDIF (CppUnit_FOUND)

########### youBot display example application ###########
add_subdirectory(display)


SET(YOUBOT_DRIVER_SRC
  ${PROJECT_SOURCE_DIR}/youbot/EthercatMaster.cpp
  ${PROJECT_SOURCE_DIR}/youbot/EthercatMasterWithThread.cpp
  ${PROJECT_SOURCE_DIR}/youbot/EthercatMasterWithoutThread.cpp
  ${PROJECT_SOURCE_DIR}/generic/Logger.cpp
  ${PROJECT_SOURCE_DIR}/generic/ConfigFile.cpp
  ${PROJECT_SOURCE_DIR}/generic/PidController.cpp
  ${PROJECT_SOURCE_DIR}/generic-joint/JointData.cpp
  ${PROJECT_SOURCE_DIR}/generic-joint/JointTrajectory.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotJoint.cpp
  ${PROJECT_SOURCE_DIR}/youbot/JointLimitMonitor.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotBase.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotManipulator.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotGripper.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotGripperBar.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotGripperParameter.cpp
  ${PROJECT_SOURCE_DIR}/youbot/DataTrace.cpp
  ${PROJECT_SOURCE_DIR}/youbot/GripperDataTrace.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotJointParameter.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotJointParameterReadOnly.cpp
  ${PROJECT_SOURCE_DIR}/youbot/YouBotJointParameterPasswordProtected.cpp
  ${PROJECT_SOURCE_DIR}/youbot/JointTrajectoryController.cpp
  ${PROJECT_SOURCE_DIR}/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  ${PROJECT_SOURCE_DIR}/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)


SET(YOUBOT_DRIVER_HEADERS
  youbot/EthercatMaster.hpp
  youbot/EthercatMasterInterface.hpp
  youbot/EthercatMasterWithThread.hpp
  youbot/EthercatMasterWithoutThread.hpp
  youbot/YouBotGripper.hpp
  youbot/YouBotGripperBar.hpp
  youbot/YouBotGripperParameter.hpp
  youbot/YouBotBase.hpp
  youbot/YouBotManipulator.hpp
  youbot/YouBotJoint.hpp
  youbot/JointLimitMonitor.hpp
  youbot/YouBotJointParameterReadOnly.hpp
	youbot/YouBotJointParameterPasswordProtected.hpp
  youbot/YouBotJointParameter.hpp
  youbot/YouBotSlaveMailboxMsg.hpp
  youbot/YouBotSlaveMsg.hpp
  youbot/ProtocolDefinitions.hpp
  youbot/YouBotJointStorage.hpp
  youbot/DataTrace.hpp
  youbot/GripperDataTrace.hpp
  youbot/JointTrajectoryController.hpp
  base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
  base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
  base-kinematic/WheeledBaseKinematic.hpp
  generic/Exceptions.hpp
  generic/Logger.hpp
  generic/Time.hpp
  generic/Units.hpp
  generic/ConfigFile.hpp
  generic/PidController.hpp
  generic/dataobjectlockfree/DataObjectLockFree.hpp
  generic/dataobjectlockfree/target.hpp
  generic/dataobjectlockfree/os/oro_arch.h
  generic/dataobjectlockfree/os/oro_gcc/oro_arch.h
  generic/dataobjectlockfree/os/oro_i386/oro_arch.h
  generic/dataobjectlockfree/os/oro_msvc/oro_arch.h
  generic/dataobjectlockfree//os/oro_powerpc/oro_arch.h  
  generic/dataobjectlockfree/os/oro_powerpc/oro_atomic.h  
  generic/dataobjectlockfree/os/oro_powerpc/oro_system.h
  generic/dataobjectlockfree/os/oro_x86_64/oro_arch.h
  generic-gripper/Gripper.hpp
  generic-gripper/GripperData.hpp
  generic-gripper/GripperParameter.hpp
  generic-joint/Joint.hpp
  generic-joint/JointData.hpp
  generic-joint/JointTrajectory.hpp
  generic-joint/JointParameter.hpp
  one-dof-gripper/OneDOFGripper.hpp
  one-dof-gripper/OneDOFGripperData.hpp
)

SET(YOUBOT_DRIVER_CONFIG
  ${PROJECT_SOURCE_DIR}/config/youbot-ethercat.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-base.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-manipulator.cfg
)

IF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)
  rosbuild_add_library(YouBotDriver ${YOUBOT_DRIVER_SRC})
ELSE(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)
  ADD_LIBRARY(YouBotDriver ${YOUBOT_DRIVER_SRC})
ENDIF(DEFINED ROS_ROOT_PATH AND USE_ROS MATCHES ON)

TARGET_LINK_LIBRARIES(YouBotDriver soem)
ADD_LIBRARY(YouBotDriverShared SHARED "")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES OUTPUT_NAME "YouBotDriver")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(YouBotDriverShared -Wl,-whole-archive YouBotDriver soem -Wl,-no-whole-archive)

MACRO(INSTALL_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    INSTALL(FILES ${HEADER} DESTINATION include/youbot/youbot/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(INSTALL_HEADERS_WITH_DIRECTORY)

MACRO(INSTALL_CONFIG_WITH_DIRECTORY CONFIG_LIST)
  FOREACH(CONFIG ${${CONFIG_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${CONFIG})
    INSTALL(FILES ${CONFIG} DESTINATION config)
  ENDFOREACH(CONFIG)
ENDMACRO(INSTALL_CONFIG_WITH_DIRECTORY)

INSTALL_HEADERS_WITH_DIRECTORY(YOUBOT_DRIVER_HEADERS)
INSTALL_CONFIG_WITH_DIRECTORY(YOUBOT_DRIVER_CONFIG)
INSTALL(TARGETS YouBotDriver YouBotDriverShared RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)


#ADD_EXECUTABLE(YouBot_test
#  ${PROJECT_SOURCE_DIR}/Test_commutation.cpp
#)

#target_link_libraries(YouBot_test YouBotDriver)

