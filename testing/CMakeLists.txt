cmake_minimum_required(VERSION 2.8)

ADD_EXECUTABLE(base_arm_gripper_test
  ${PROJECT_SOURCE_DIR}/testing/TestSuite.cpp
  ${PROJECT_SOURCE_DIR}/testing/YouBotBaseTest.cpp
  ${PROJECT_SOURCE_DIR}/testing/YouBotBaseTestWithoutThread.cpp
  ${PROJECT_SOURCE_DIR}/testing/YouBotBaseKinematicsTest.cpp
  ${PROJECT_SOURCE_DIR}/testing/YouBotArmTest.cpp
  ${PROJECT_SOURCE_DIR}/testing/YouBotArmTestWithoutThread.cpp
  ${PROJECT_SOURCE_DIR}/testing/YouBotGripperTest.cpp
)

target_link_libraries(base_arm_gripper_test YouBotDriver cppunit)

INSTALL(TARGETS base_arm_gripper_test RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

