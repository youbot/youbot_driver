cmake_minimum_required(VERSION 2.8)

ADD_EXECUTABLE(displayIpAddress
  ${CMAKE_HOME_DIRECTORY}/display/DisplayIpAddress.cpp

)

target_link_libraries(displayIpAddress stdc++)

INSTALL(TARGETS displayIpAddress RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

