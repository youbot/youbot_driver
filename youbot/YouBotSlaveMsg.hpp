/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/

#ifndef _YOUBOT_SLAVE_MESSAGE_H
#define	_YOUBOT_SLAVE_MESSAGE_H

#include <ethercattype.h>
#include <string>
#include <time.h>

namespace youbot {

	/// Output part from the EtherCat message of the youBot EtherCat slaves

	struct OutputBuffer {
		int32 positionOrSpeed;
		uint8 controllerMode;
	} __attribute__((__packed__));


	/// Output part from the EtherCat message of the youBot EtherCat slaves

	struct InputBuffer {
		int32 actualPosition;
		int32 actualCurrent;
		int32 actualVelocity;
		uint16 errorFlags;
		uint16 driverTemperature;
	} __attribute__((__packed__));

        ///////////////////////////////////////////////////////////////////////////////
	/// EtherCat message of the youBot EtherCat slaves
        ///////////////////////////////////////////////////////////////////////////////
	class YouBotSlaveMsg {
	public:


		OutputBuffer stctOutput;
		InputBuffer stctInput;
		unsigned int jointNumber;

		// Constructor

		YouBotSlaveMsg() {
			stctOutput.controllerMode = 0;
			stctOutput.positionOrSpeed = 0;
			stctInput.actualCurrent = 0;
			stctInput.actualPosition = 0;
			stctInput.actualVelocity = 0;
			stctInput.driverTemperature = 0;
			stctInput.errorFlags = 0;
			jointNumber = 0;
		}

		// Copy-Constructor

		YouBotSlaveMsg(const YouBotSlaveMsg &copy) {
			stctOutput = copy.stctOutput;
			stctInput = copy.stctInput;
			jointNumber = copy.jointNumber;
		}

		// Destructor

		~YouBotSlaveMsg() {
		}

		// assignment operator

		YouBotSlaveMsg & operator=(const YouBotSlaveMsg &copy) {
			stctOutput = copy.stctOutput;
			stctInput = copy.stctInput;
			jointNumber = copy.jointNumber;

			return *this;
		}
	};

} // namespace youbot

#endif	/* _YOUBOT_SLAVE_MESSAGE_H */
