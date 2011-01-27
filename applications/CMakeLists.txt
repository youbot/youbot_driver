

ADD_EXECUTABLE(YouBot_test
  ${CMAKE_HOME_DIRECTORY}/applications/Test.cpp
)

ADD_EXECUTABLE(YouBot_KeyboardRemoteControl
  ${CMAKE_HOME_DIRECTORY}/applications/KeyboardRemoteControl.cpp
)

target_link_libraries(YouBot_test YouBotDriver)
target_link_libraries(YouBot_KeyboardRemoteControl YouBotDriver ncurses)

INSTALL(TARGETS YouBot_test YouBot_KeyboardRemoteControl RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

