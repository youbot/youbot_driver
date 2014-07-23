include_directories(
	${PROJECT_SOURCE_DIR}/include/youbot_driver/soem
	${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/osal
	${PROJECT_SOURCE_DIR}/include/youbot_driver/soem/oshw/linux
)

ADD_LIBRARY(soem 
	ethercatbase.c 
	ethercatcoe.c 
	ethercatconfig.c 
	ethercatdc.c 
	ethercatfoe.c 
	ethercatmain.c 
	ethercatprint.c 
	ethercatsoe.c 
	osal/linux/osal.c
	oshw/linux/nicdrv.c
	oshw/linux/oshw.c
)

INSTALL(TARGETS soem 
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

