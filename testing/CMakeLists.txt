cmake_minimum_required(VERSION 2.8)

ADD_EXECUTABLE(base_arm_gripper_test
  ${CMAKE_HOME_DIRECTORY}/testing/TestSuite.cpp
  ${CMAKE_HOME_DIRECTORY}/testing/YouBotBaseTest.cpp
	${CMAKE_HOME_DIRECTORY}/testing/YouBotBaseTestWithoutThread.cpp
  ${CMAKE_HOME_DIRECTORY}/testing/YouBotBaseKinematicsTest.cpp
  ${CMAKE_HOME_DIRECTORY}/testing/YouBotArmTest.cpp
	${CMAKE_HOME_DIRECTORY}/testing/YouBotGripperTest.cpp
)

target_link_libraries(base_arm_gripper_test YouBotDriver cppunit)

INSTALL(TARGETS base_arm_gripper_test RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

