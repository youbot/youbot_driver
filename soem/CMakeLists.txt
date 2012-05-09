ADD_LIBRARY(soem 
	src/ethercatbase.c 
  src/ethercatcoe.c 
  src/ethercatconfig.c 
  src/ethercatfoe.c 
  src/ethercatsoe.c 
  src/ethercatmain.c 
  src/ethercatprint.c 
  src/nicdrv.c 
  src/ethercatdc.c
)

SET(SOEM_HEADERS
	src/ethercatbase.h
	src/ethercatcoe.h
	src/ethercatconfig.h
	src/ethercatconfiglist.h
	src/ethercatdc.h
	src/ethercatfoe.h
	src/ethercatmain.h
	src/ethercatprint.h
	src/ethercatsoe.h
	src/ethercattype.h
	src/nicdrv.h
)


MACRO(INSTALL_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    INSTALL(FILES ${HEADER} DESTINATION include/youbot/youbot/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(INSTALL_HEADERS_WITH_DIRECTORY)

INSTALL_HEADERS_WITH_DIRECTORY(SOEM_HEADERS)

INSTALL(TARGETS soem RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

