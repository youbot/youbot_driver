cmake_minimum_required(VERSION 2.8)

add_definitions(-DCONFIG_FOLDER_PATH="${CMAKE_CURRENT_LIST_DIR}/../../config/")

ADD_EXECUTABLE(base_arm_gripper_test
  TestSuite.cpp
  YouBotBaseTest.cpp
  YouBotBaseTestWithoutThread.cpp
  YouBotBaseKinematicsTest.cpp
  YouBotArmTest.cpp
  YouBotArmTestWithoutThread.cpp
  YouBotGripperTest.cpp
)

target_link_libraries(base_arm_gripper_test YouBotDriver cppunit)

INSTALL(TARGETS base_arm_gripper_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

