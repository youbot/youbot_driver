PROJECT(YouBot_Driver)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_C_FLAGS_DEBUG "-Wall -g")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")

SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib) #static libraries
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin) #executables
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib) #dynamic libraries (plugins)

INCLUDE_DIRECTORIES(${CMAKE_HOME_DIRECTORY})

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


FIND_PACKAGE(Soem REQUIRED)
IF (SOEM_FOUND)
  INCLUDE_DIRECTORIES(${SOEM_INCLUDE_DIR})
  LINK_DIRECTORIES(${SOEM_LIBRARY_DIR})
  LINK_LIBRARIES(${SOEMLIB})
	MESSAGE(STATUS "SUCCESSFUL: Soem found at "${SOEM_LIBRARY_DIR})
ELSE (SOEM_FOUND)
	MESSAGE(STATUS "ERROR: Soem not found")
ENDIF (SOEM_FOUND)

SET(DOXYFILE_LATEX "NO")
SET(DOXYFILE_OUTPUT_DIR ${CMAKE_HOME_DIRECTORY}/doc)
include(UseDoxygen)


add_subdirectory(youbot)
#add_subdirectory(testing)


#ADD_EXECUTABLE(YouBot_test
#  ${CMAKE_HOME_DIRECTORY}/Test.cpp
#)

#target_link_libraries(YouBot_test YouBotDriver)

