#
# Helper CMake file to define path to config files
#
#
# Output:
#    youbot_driver_CONFIG_DIR - Path to config files such as ethercat.cfg
#

if (youbot_driver_SOURCE_PREFIX)
	set(youbot_driver_CONFIG_DIR ${youbot_driver_SOURCE_PREFIX}/config)
else (youbot_driver_SOURCE_PREFIX)
	set(youbot_driver_CONFIG_DIR ${youbot_driver_INSTALL_PREFIX}/share/youbot_driver/config)
endif (youbot_driver_SOURCE_PREFIX)

