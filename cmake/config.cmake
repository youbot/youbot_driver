#
# Helper CMake file to define path to config files and additional include dirs
#
#
# Output:
#    youbot_driver_CONFIG_DIR - Path to config files such as ethercat.cfg
#    youbot_driver_INCLUDE_DIRS - Appended with os-specific include dirs (only for install target)
#

if (youbot_driver_SOURCE_PREFIX)
	set(youbot_driver_CONFIG_DIR ${youbot_driver_SOURCE_PREFIX}/config)
else (youbot_driver_SOURCE_PREFIX)
	set(youbot_driver_CONFIG_DIR ${youbot_driver_INSTALL_PREFIX}/share/youbot_driver/config)

	# add additional include dirs for install
	# otherwise catkin does provide only the main "include" folder
	if(WIN32)
		list(APPEND youbot_driver_INCLUDE_DIRS
			${youbot_driver_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/youbot_driver/soem/osal
			${youbot_driver_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/youbot_driver/soem/osal/win32
			${youbot_driver_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/youbot_driver/soem/oshw/win32
		)
	else(WIN32)
		list(APPEND youbot_driver_INCLUDE_DIRS
			${youbot_driver_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/youbot_driver/soem/osal
			${youbot_driver_INSTALL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/youbot_driver/soem/oshw/linux
		)
	endif(WIN32)
endif (youbot_driver_SOURCE_PREFIX)

