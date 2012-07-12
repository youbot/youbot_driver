cmake_minimum_required(VERSION 2.8)

ADD_EXECUTABLE(test_suite
  ${CMAKE_HOME_DIRECTORY}/testing/TestSuite.cpp
  ${CMAKE_HOME_DIRECTORY}/testing/YouBotBaseTest.cpp
	${CMAKE_HOME_DIRECTORY}/testing/YouBotBaseTestWithoutThread.cpp
  ${CMAKE_HOME_DIRECTORY}/testing/YouBotBaseKinematicsTest.cpp
  ${CMAKE_HOME_DIRECTORY}/testing/YouBotArmTest.cpp
)

target_link_libraries(test_suite YouBotDriver cppunit)

INSTALL(TARGETS test_suite RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

