


IF(USE_ETHERCAT_MASTER_WITH_THREAD)
  SET(YOUBOT_DRIVER_SRC
    ${CMAKE_HOME_DIRECTORY}/youbot/EthercatMaster.cpp
    ${CMAKE_HOME_DIRECTORY}/generic/Logger.cpp
    ${CMAKE_HOME_DIRECTORY}/generic/ConfigFile.cpp
    ${CMAKE_HOME_DIRECTORY}/generic-joint/JointData.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJoint.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotBase.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotManipulator.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotGripper.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotGripperParameter.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/MotorProtection.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/DataTrace.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameter.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameterReadOnly.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameterPasswordProtected.cpp
    ${CMAKE_HOME_DIRECTORY}/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
    ${CMAKE_HOME_DIRECTORY}/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
  )
ELSE(USE_ETHERCAT_MASTER_WITH_THREAD)
  SET(YOUBOT_DRIVER_SRC
    ${CMAKE_HOME_DIRECTORY}/youbot/EthercatMasterWithoutThread.cpp
    ${CMAKE_HOME_DIRECTORY}/generic/Logger.cpp
    ${CMAKE_HOME_DIRECTORY}/generic/ConfigFile.cpp
    ${CMAKE_HOME_DIRECTORY}/generic-joint/JointData.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJoint.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotBase.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotManipulator.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotGripper.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotGripperParameter.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/MotorProtection.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/DataTrace.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameter.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameterReadOnly.cpp
    ${CMAKE_HOME_DIRECTORY}/youbot/YouBotJointParameterPasswordProtected.cpp
    ${CMAKE_HOME_DIRECTORY}/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
    ${CMAKE_HOME_DIRECTORY}/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
  )
ENDIF(USE_ETHERCAT_MASTER_WITH_THREAD)




SET(YOUBOT_DRIVER_HEADERS
  EthercatMaster.hpp
  EthercatMasterWithoutThread.hpp
  YouBotGripper.hpp
  YouBotGripperParameter.hpp
  YouBotBase.hpp
  YouBotManipulator.hpp
  YouBotJoint.hpp
  YouBotJointParameterReadOnly.hpp
	YouBotJointParameterPasswordProtected.hpp
  YouBotJointParameter.hpp
  YouBotSlaveMailboxMsg.hpp
  YouBotSlaveMsg.hpp
  ProtocolDefinitions.hpp
  YouBotJointStorage.hpp
  MotorProtection.hpp
  DataTrace.hpp
../base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
../base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
../base-kinematic/WheeledBaseKinematic.hpp

../generic/Exceptions.hpp
../generic/Logger.hpp
../generic/Time.hpp
../generic/Units.hpp
../generic/ConfigFile.hpp

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

TARGET_LINK_LIBRARIES(YouBotDriver soem)

ADD_LIBRARY(YouBotDriverShared SHARED)
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES OUTPUT_NAME "YouBotDriver")
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

