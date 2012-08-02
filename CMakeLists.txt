PROJECT(YouBot_Driver)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
SET(CMAKE_CXX_FLAGS_DEBUG "-fPIC -Wall -g")

SET(CMAKE_C_FLAGS_RELEASE "-fPIC -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-fPIC -Wall")

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake-modules")

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #static libraries
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) #executables
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) #dynamic libraries (plugins)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

# find required and optional dependencies via Find<name>.cmake modules

#FIND_PACKAGE(CppUnit)
#IF(CppUnit_FOUND)
#	MESSAGE(STATUS "SUCCESSFUL: CPP_UNIT found")
#ELSE(CppUnit_FOUND)
#	MESSAGE(STATUS "WARNING: CPP_UNIT not found")
#ENDIF (CppUnit_FOUND)

#SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)
IF (Boost_FOUND)
  LINK_LIBRARIES(boost_thread boost_date_time boost_filesystem boost_system boost_regex)
MESSAGE(STATUS "SUCCESSFUL: Boost found")
ELSE (Boost_FOUND)
MESSAGE(STATUS "ERROR: Boost not found")
ENDIF (Boost_FOUND)

add_subdirectory(soem)
INCLUDE_DIRECTORIES(
 ${PROJECT_SOURCE_DIR}
 ${PROJECT_SOURCE_DIR}/soem/src
)

SET(DOXYFILE_LATEX "NO")
SET(DOXYFILE_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/doc)
include(UseDoxygen)


add_subdirectory(youbot)
add_subdirectory(testing)
add_subdirectory(display)


#ADD_EXECUTABLE(YouBot_test
#  ${PROJECT_SOURCE_DIR}/Test.cpp
#)

#target_link_libraries(YouBot_test YouBotDriver)

#ADD_EXECUTABLE(PositionTest
#  ${PROJECT_SOURCE_DIR}/PositionTest.cpp
#)

#target_link_libraries(PositionTest YouBotDriver)


#ADD_EXECUTABLE(VelocityTest
#  ${PROJECT_SOURCE_DIR}/VelocityTest.cpp
#)

#target_link_libraries(VelocityTest YouBotDriver)


#ADD_EXECUTABLE(CurrentTest
#  ${PROJECT_SOURCE_DIR}/CurrentTest.cpp
#)

#target_link_libraries(CurrentTest YouBotDriver)


