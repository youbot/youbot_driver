include_directories(${PROJECT_SOURCE_DIR}/include/youbot_driver/soem)

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

INSTALL(TARGETS soem 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

