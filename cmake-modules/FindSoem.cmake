# - Try to find soem
# Once done this will define
#
#  SOEM_FOUND - soem found
#  SOEM_INCLUDE_DIR - the soem include directory
#  SOEM_LIBRARY_DIR - soem lib directory
#

SET(SOEMLIB "soem")

FIND_PATH(SOEM_INCLUDE_DIR NAMES ethercatmain.h
  PATHS
  $ENV{ROBOTPKG_BASE}/include/soem/src 
  ${SOEM_PATH}/include/soem/src
	${soem_PACKAGE_PATH}/src
  ENV CPATH
  /usr/include/
  /usr/include/soem/src
  /usr/local/include/
  /usr/local/include/soem/src
  /opt/local/include/
  /opt/local/include/soem/src
  NO_DEFAULT_PATH
)

#MARK_AS_ADVANCED("SOEM_INCLUDE_DIR: "${SOEM_INCLUDE_DIR})

FIND_LIBRARY(SOEM_LIBRARY_TMP NAMES ${SOEMLIB} "SoemLibraries"
  PATHS
  $ENV{ROBOTPKG_BASE}/lib
  ${SOEM_PATH}/lib
	${soem_PACKAGE_PATH}/lib
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  NO_DEFAULT_PATH
)

IF(SOEM_LIBRARY_TMP)
  GET_FILENAME_COMPONENT( SOEM_LIBRARY_DIR ${SOEM_LIBRARY_TMP} PATH )
ENDIF(SOEM_LIBRARY_TMP)

IF(SOEM_INCLUDE_DIR AND SOEM_LIBRARY_DIR)
  SET(SOEM_FOUND TRUE)
ENDIF(SOEM_INCLUDE_DIR AND SOEM_LIBRARY_DIR)

# show the SOEM_INCLUDE_DIR and SOEM_LIBRARY_DIR variables only in the advanced view
IF(SOEM_FOUND)
  MARK_AS_ADVANCED(SOEM_INCLUDE_DIR SOEM_LIBRARY_DIR )
ENDIF(SOEM_FOUND)


