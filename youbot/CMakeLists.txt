

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
  EthercatMaster.hpp
  EthercatMasterInterface.hpp
  EthercatMasterWithThread.hpp
  EthercatMasterWithoutThread.hpp
  YouBotGripper.hpp
  YouBotGripperBar.hpp
  YouBotGripperParameter.hpp
  YouBotBase.hpp
  YouBotManipulator.hpp
  YouBotJoint.hpp
  JointLimitMonitor.hpp
  YouBotJointParameterReadOnly.hpp
	YouBotJointParameterPasswordProtected.hpp
  YouBotJointParameter.hpp
  YouBotSlaveMailboxMsg.hpp
  YouBotSlaveMsg.hpp
  ProtocolDefinitions.hpp
  YouBotJointStorage.hpp
  DataTrace.hpp
  GripperDataTrace.hpp
  JointTrajectoryController.hpp
../base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp
../base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp
../base-kinematic/WheeledBaseKinematic.hpp

../generic/Exceptions.hpp
../generic/Logger.hpp
../generic/Time.hpp
../generic/Units.hpp
../generic/ConfigFile.hpp
../generic/PidController.hpp

../generic/dataobjectlockfree/DataObjectLockFree.hpp
../generic/dataobjectlockfree/target.hpp
../generic/dataobjectlockfree/os/oro_arch.h
../generic/dataobjectlockfree/os/oro_gcc/oro_arch.h
../generic/dataobjectlockfree/os/oro_i386/oro_arch.h
../generic/dataobjectlockfree/os/oro_msvc/oro_arch.h
../generic/dataobjectlockfree//os/oro_powerpc/oro_arch.h  
../generic/dataobjectlockfree/os/oro_powerpc/oro_atomic.h  
../generic/dataobjectlockfree/os/oro_powerpc/oro_system.h
../generic/dataobjectlockfree/os/oro_x86_64/oro_arch.h

../generic-gripper/Gripper.hpp
../generic-gripper/GripperData.hpp
../generic-gripper/GripperParameter.hpp

../generic-joint/Joint.hpp
../generic-joint/JointData.hpp
../generic-joint/JointTrajectory.hpp
../generic-joint/JointParameter.hpp

../one-dof-gripper/OneDOFGripper.hpp
../one-dof-gripper/OneDOFGripperData.hpp

)


SET(YOUBOT_DRIVER_CONFIG
  ${PROJECT_SOURCE_DIR}/config/youbot-ethercat.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-base.cfg
  ${PROJECT_SOURCE_DIR}/config/youbot-manipulator.cfg
)

IF(DEFINED ROS_ROOT_PATH)
  rosbuild_add_library(YouBotDriver ${YOUBOT_DRIVER_SRC})
ELSE(DEFINED ROS_ROOT_PATH)
  ADD_LIBRARY(YouBotDriver ${YOUBOT_DRIVER_SRC})
ENDIF(DEFINED ROS_ROOT_PATH)

TARGET_LINK_LIBRARIES(YouBotDriver soem)
ADD_LIBRARY(YouBotDriverShared SHARED)
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES OUTPUT_NAME "YouBotDriver")
SET_TARGET_PROPERTIES(YouBotDriverShared PROPERTIES LINKER_LANGUAGE C)
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

