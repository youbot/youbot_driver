cmake_minimum_required(VERSION 2.8)

ADD_EXECUTABLE(displayIpAddress
  DisplayIpAddress.cpp

)

INSTALL(TARGETS displayIpAddress
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

