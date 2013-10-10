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
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
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
add_subdirectory(src/soem)

INCLUDE_DIRECTORIES(
 ${PROJECT_SOURCE_DIR}/include
 ${PROJECT_SOURCE_DIR}/include/soem
 ${PROJECT_SOURCE_DIR}/include/generic
 ${PROJECT_SOURCE_DIR}/include/test
)

########### CppUnit and tests ###########
FIND_PACKAGE(CppUnit)
IF(CppUnit_FOUND)
	MESSAGE(STATUS "SUCCESSFUL: CPP_UNIT found")
  add_subdirectory(src/test)
#ELSE(CppUnit_FOUND)
#	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
ENDIF (CppUnit_FOUND)

########### youBot display example application ###########
add_subdirectory(src/display)


SET(YOUBOT_DRIVER_SRC
  ${PROJECT_SOURCE_DIR}/src/youbot/EthercatMaster.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/EthercatMasterWithThread.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/EthercatMasterWithoutThread.cpp
  ${PROJECT_SOURCE_DIR}/src/generic/Logger.cpp
  ${PROJECT_SOURCE_DIR}/src/generic/ConfigFile.cpp
  ${PROJECT_SOURCE_DIR}/src/generic/PidController.cpp
  ${PROJECT_SOURCE_DIR}/src/generic-joint/JointData.cpp
  ${PROJECT_SOURCE_DIR}/src/generic-joint/JointTrajectory.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJoint.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/JointLimitMonitor.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotBase.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotManipulator.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotGripper.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotGripperBar.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotGripperParameter.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/DataTrace.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/GripperDataTrace.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJointParameter.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJointParameterReadOnly.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/YouBotJointParameterPasswordProtected.cpp
  ${PROJECT_SOURCE_DIR}/src/youbot/JointTrajectoryController.cpp
  ${PROJECT_SOURCE_DIR}/src/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  ${PROJECT_SOURCE_DIR}/src/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)


SET(YOUBOT_DRIVER_HEADERS
  include/youbot/EthercatMaster.hpp
  include/youbot/EthercatMasterInterface.hpp
  include/youbot/EthercatMasterWithThread.hpp
  include/youbot/EthercatMasterWithoutThread.hpp
  include/youbot/YouBotGripper.hpp
  include/youbot/YouBotGripperBar.hpp
  include/youbot/YouBotGripperParameter.hpp
  include/youbot/YouBotBase.hpp
  include/youbot/YouBotManipulator.hpp
  include/youbot/YouBotJoint.hpp
  include/youbot/JointLimitMonitor.hpp
  include/youbot/YouBotJointParameterReadOnly.hpp
	include/youbot/YouBotJointParameterPasswordProtected.hpp
  include/youbot/YouBotJointParameter.hpp
  include/youbot/YouBotSlaveMailboxMsg.hpp
  include/youbot/YouBotSlaveMsg.hpp
  include/youbot/ProtocolDefinitions.hpp
  include/youbot/YouBotJointStorage.hpp
  include/youbot/DataTrace.hpp
  include/youbot/GripperDataTrace.hpp
  include/youbot/JointTrajectoryController.hpp
  include/base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
  include/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
  include/base-kinematic/WheeledBaseKinematic.hpp
  include/generic/Exceptions.hpp
  include/generic/Logger.hpp
  include/generic/Time.hpp
  include/generic/Units.hpp
  include/generic/ConfigFile.hpp
  include/generic/PidController.hpp
  include/generic/dataobjectlockfree/DataObjectLockFree.hpp
  include/generic/dataobjectlockfree/target.hpp
  include/generic/dataobjectlockfree/os/oro_arch.h
  include/generic/dataobjectlockfree/os/oro_gcc/oro_arch.h
  include/generic/dataobjectlockfree/os/oro_i386/oro_arch.h
  include/generic/dataobjectlockfree/os/oro_msvc/oro_arch.h
  include/generic/dataobjectlockfree//os/oro_powerpc/oro_arch.h  
  include/generic/dataobjectlockfree/os/oro_powerpc/oro_atomic.h  
  include/generic/dataobjectlockfree/os/oro_powerpc/oro_system.h
  include/generic/dataobjectlockfree/os/oro_x86_64/oro_arch.h
  include/generic-gripper/Gripper.hpp
  include/generic-gripper/GripperData.hpp
  include/generic-gripper/GripperParameter.hpp
  include/generic-joint/Joint.hpp
  include/generic-joint/JointData.hpp
  include/generic-joint/JointTrajectory.hpp
  include/generic-joint/JointParameter.hpp
  include/one-dof-gripper/OneDOFGripper.hpp
  include/one-dof-gripper/OneDOFGripperData.hpp
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

