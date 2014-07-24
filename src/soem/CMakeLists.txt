if(WIN32)
	include_directories(
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/osal
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/osal/win32
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/oshw/win32
	)
else(WIN32)
	include_directories(
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/osal
		${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/oshw/linux
	)
endif(WIN32)

if(WIN32)
	set(SOEM_OS_SOURCE_FILES
		osal/win32/osal.c
		oshw/win32/nicdrv.c
		oshw/win32/oshw.c
	)		
else(WIN32)
	set(SOEM_OS_SOURCE_FILES
		osal/linux/osal.c
		oshw/linux/nicdrv.c
		oshw/linux/oshw.c
	)		
endif(WIN32)

ADD_LIBRARY(soem 
	ethercatbase.c 
	ethercatcoe.c 
	ethercatconfig.c 
	ethercatdc.c 
	ethercatfoe.c 
	ethercatmain.c 
	ethercatprint.c 
	ethercatsoe.c 
	${SOEM_OS_SOURCE_FILES}
)

INSTALL(TARGETS soem 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

