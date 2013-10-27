INCLUDE_DIRECTORIES(
 ${PROJECT_SOURCE_DIR}/include/soem
)

ADD_LIBRARY(soem 
	ethercatbase.c 
  ethercatcoe.c 
  ethercatconfig.c 
  ethercatfoe.c 
  ethercatsoe.c 
  ethercatmain.c 
  ethercatprint.c 
  nicdrv.c 
  ethercatdc.c
)

SET(SOEM_HEADERS
	ethercatbase.h
	ethercatcoe.h
	ethercatconfig.h
	ethercatconfiglist.h
	ethercatdc.h
	ethercatfoe.h
	ethercatmain.h
	ethercatprint.h
	ethercatsoe.h
	ethercattype.h
	nicdrv.h
)


MACRO(INSTALL_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    INSTALL(FILES ${HEADER} DESTINATION include/youbot/youbot/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(INSTALL_HEADERS_WITH_DIRECTORY)

INSTALL_HEADERS_WITH_DIRECTORY(SOEM_HEADERS)

INSTALL(TARGETS soem RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

