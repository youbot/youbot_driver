
SET( YOUBOT_DRIVER_VERSION 0.01 )

FIND_PACKAGE(CppUnit)
IF(CppUnit_FOUND)
	MESSAGE(STATUS "SUCCESSFUL: CPP_UNIT found")
ELSE(CppUnit_FOUND)
	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
ENDIF (CppUnit_FOUND)

#SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)
IF (Boost_FOUND)
  LINK_LIBRARIES(boost_thread boost_date_time boost_filesystem boost_system boost_regex)
MESSAGE(STATUS "SUCCESSFUL: Boost found")
ELSE (Boost_FOUND)
MESSAGE(STATUS "ERROR: Boost not found")
ENDIF (Boost_FOUND)

FIND_PACKAGE(BoostLog)
IF (BOOSTLOG_FOUND)
  INCLUDE_DIRECTORIES(${BOOSTLOG_INCLUDE_DIR})
  LINK_DIRECTORIES(${BOOSTLOG_LIBRARY_DIR})
  LINK_LIBRARIES(${BOOSTLOGLIB})
  ADD_DEFINITIONS(-DBOOST_LOG_FOUND)
MESSAGE(STATUS "SUCCESSFUL: boost::log found at "${BOOSTLOG_LIBRARY_DIR})
ELSE (BOOSTLOG_FOUND)
MESSAGE(STATUS "ERROR: boost::log not found")
ENDIF (BOOSTLOG_FOUND)


FIND_PACKAGE(Soem REQUIRED)
IF (SOEM_FOUND)
  INCLUDE_DIRECTORIES(${SOEM_INCLUDE_DIR})
  LINK_DIRECTORIES(${SOEM_LIBRARY_DIR})
  LINK_LIBRARIES(${SOEMLIB})
	MESSAGE(STATUS "SUCCESSFUL: Soem found at "${SOEM_LIBRARY_DIR})
ELSE (SOEM_FOUND)
	MESSAGE(STATUS "ERROR: Soem not found")
ENDIF (SOEM_FOUND)


FIND_PACKAGE(Rudeconfig REQUIRED)
IF (RUDECONFIG_FOUND)
  INCLUDE_DIRECTORIES(${RUDECONFIG_INCLUDE_DIR})
  LINK_DIRECTORIES(${RUDECONFIG_LIBRARY_DIR})
  LINK_LIBRARIES(${RUDECONFIGLIB})
	MESSAGE(STATUS "SUCCESSFUL: rudeconfig found at "${RUDECONFIG_LIBRARY_DIR})
ELSE (RUDECONFIG_FOUND)
	MESSAGE(STATUS "ERROR: rudeconfig not found")
ENDIF (RUDECONFIG_FOUND)



SET(YOUBOT_DRIVER_SRC
  ${CMAKE_HOME_DIRECTORY}/generic/Errors.cpp
  ${CMAKE_HOME_DIRECTORY}/generic/Logger.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/EthercatMaster.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJoint.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotBase.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotManipulator.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotGripper.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotGripperParameter.cpp
  
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameter.cpp
  ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameterReadOnly.cpp
  ${CMAKE_HOME_DIRECTORY}/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  ${CMAKE_HOME_DIRECTORY}/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)

SET(YOUBOT_DRIVER_HEADERS
  EthercatMaster.hpp
  YouBotGripper.hpp
  YouBotGripperParameter.hpp
  YouBotBase.hpp
  YouBotManipulator.hpp
  YouBotJoint.hpp
  YouBotJointParameterReadOnly.hpp
  YouBotJointParameter.hpp
  YouBotSlaveMailboxMsg.hpp
  YouBotSlaveMsg.hpp
  ProtocolDefinitions.hpp
  YouBotJointStorage.hpp
../base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
../base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
../base-kinematic/WheeledBaseKinematic.hpp

../generic/Errors.hpp
../generic/Exceptions.hpp
../generic/Logger.hpp
../generic/Time.hpp
../generic/Units.hpp

../generic-gripper/Gripper.hpp
../generic-gripper/GripperData.hpp
../generic-gripper/GripperParameter.hpp

../generic-joint/Joint.hpp
../generic-joint/JointData.hpp
../generic-joint/JointParameter.hpp

../one-dof-gripper/OneDOFGripper.hpp
../one-dof-gripper/OneDOFGripperData.hpp

)


SET(YOUBOT_DRIVER_CONFIG
  ${CMAKE_HOME_DIRECTORY}/config/youbot-ethercat.cfg
  ${CMAKE_HOME_DIRECTORY}/config/youbot-base.cfg
  ${CMAKE_HOME_DIRECTORY}/config/youbot-manipulator.cfg
  ${CMAKE_HOME_DIRECTORY}/config/applications.cfg
)

ADD_LIBRARY(YouBotDriver
  ${YOUBOT_DRIVER_SRC}
)


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
INSTALL(TARGETS YouBotDriver RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

