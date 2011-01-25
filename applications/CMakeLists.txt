
FIND_PACKAGE(Rudeconfig REQUIRED)
IF (RUDECONFIG_FOUND)
  INCLUDE_DIRECTORIES(${RUDECONFIG_INCLUDE_DIR})
  LINK_DIRECTORIES(${RUDECONFIG_LIBRARY_DIR})
  LINK_LIBRARIES(${RUDECONFIGLIB})
	MESSAGE(STATUS "SUCCESSFUL: rudeconfig found at "${RUDECONFIG_LIBRARY_DIR})
ELSE (RUDECONFIG_FOUND)
	MESSAGE(STATUS "ERROR: rudeconfig not found")
ENDIF (RUDECONFIG_FOUND)

ADD_EXECUTABLE(YouBot_test
  ${CMAKE_HOME_DIRECTORY}/applications/Test.cpp
)

ADD_EXECUTABLE(YouBot_KeyboardRemoteControl
  ${CMAKE_HOME_DIRECTORY}/applications/KeyboardRemoteControl.cpp
)

target_link_libraries(YouBot_test YouBotDriver)
target_link_libraries(YouBot_KeyboardRemoteControl YouBotDriver ncurses)

INSTALL(TARGETS YouBot_test YouBot_KeyboardRemoteControl RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

