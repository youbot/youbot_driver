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

#ifndef _YOUBOT_SLAVE_MAILBOX_MESSAGE_H
#define	_YOUBOT_SLAVE_MAILBOX_MESSAGE_H

#include <ethercattype.h>
#include <string>
#include <time.h>

namespace youbot {
  ///////////////////////////////////////////////////////////////////////////////
	/// EtherCat mailbox message of the youBot EtherCat slaves
  ///////////////////////////////////////////////////////////////////////////////
	class YouBotSlaveMailboxMsg {
	public:

		/// Output part from the EtherCat mailbox message of the youBot EtherCat slaves
		struct mailboxOutputBuffer {
			uint8 moduleAddress; //0 = Drive  1 = Gripper
			uint8 commandNumber;
			uint8 typeNumber;
			uint8 motorNumber; //always zero
			uint32 value; //MSB first!
		} __attribute__((__packed__));

		/// Input part from the EtherCat mailbox message of the youBot EtherCat slaves
		struct mailboxInputBuffer {
			uint8 replyAddress;
			uint8 moduleAddress;
			uint8 status; //(e.g. 100 means “no error”)
			uint8 commandNumber;
			uint32 value; //MSB first!
		} __attribute__((__packed__));

		mailboxOutputBuffer stctOutput;
		mailboxInputBuffer stctInput;

		// Constructor

		YouBotSlaveMailboxMsg() {
			stctOutput.moduleAddress = 0;
			stctOutput.commandNumber = 0;
			stctOutput.typeNumber = 0;
			stctOutput.motorNumber = 0;
			stctOutput.value = 0;

			stctInput.replyAddress = 0;
			stctInput.moduleAddress = 0;
			stctInput.status = 0;
			stctInput.commandNumber = 0;
			stctInput.value = 0;
			slaveNumber = 1000;
		}

		// Constructor

		YouBotSlaveMailboxMsg(unsigned int slaveNo) {
			stctOutput.moduleAddress = 0;
			stctOutput.commandNumber = 0;
			stctOutput.typeNumber = 0;
			stctOutput.motorNumber = 0;
			stctOutput.value = 0;

			stctInput.replyAddress = 0;
			stctInput.moduleAddress = 0;
			stctInput.status = 0;
			stctInput.commandNumber = 0;
			stctInput.value = 0;
			slaveNumber = slaveNo;

		}



		// Copy-Constructor

		YouBotSlaveMailboxMsg(const YouBotSlaveMailboxMsg& copy) {
			stctOutput = copy.stctOutput;
			stctInput = copy.stctInput;
			slaveNumber = copy.slaveNumber;
			parameterName = copy.parameterName;
		}

		// Destructor

		~YouBotSlaveMailboxMsg() {
		}

		// assignment operator

		YouBotSlaveMailboxMsg & operator=(const YouBotSlaveMailboxMsg& copy) {
			stctOutput = copy.stctOutput;
			stctInput = copy.stctInput;
			slaveNumber = copy.slaveNumber;
			parameterName = copy.parameterName;

			return *this;
		}

		std::string parameterName;

		unsigned int getSlaveNo() const {
			return slaveNumber;
		}
	private:
		unsigned int slaveNumber;
	};

} // namespace youbot

#endif	/* _YOUBOT_SLAVE_MESSAGE_H */
