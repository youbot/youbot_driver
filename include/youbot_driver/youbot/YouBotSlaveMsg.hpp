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

#include "youbot_driver/generic/dataobjectlockfree/DataObjectLockFree.hpp"
#include <youbot_driver/soem/ethercattype.h>
#include <string>
#include <time.h>

namespace youbot {

	/// Output part from the EtherCat message of the youBot EtherCat slaves

	PACKED_BEGIN
	struct SlaveMessageOutput {
		int32 value;
		uint8 controllerMode;
		SlaveMessageOutput():value(0),controllerMode(0) { };
	} PACKED;
	PACKED_END


	/// Input part from the EtherCat message of the youBot EtherCat slaves

	PACKED_BEGIN
	struct SlaveMessageInput {
		int32 actualPosition;   // encoder ticks
		int32 actualCurrent;    // mA
		int32 actualVelocity;   // rpm motor axis
		uint32 errorFlags;
		int32 targetPosition;
		int32 targetCurrent;
		int32 targetVelocity;
		int32 rampGeneratorVelocity;

		SlaveMessageInput():
		actualPosition(0),actualCurrent(0),actualVelocity(0),
		errorFlags(0),targetPosition(0),targetCurrent(0),
		targetVelocity(0),rampGeneratorVelocity(0) { };
	} PACKED;
	PACKED_BEGIN

  ///////////////////////////////////////////////////////////////////////////////
	/// EtherCat message of the youBot EtherCat slaves
  ///////////////////////////////////////////////////////////////////////////////
	class YouBotSlaveMsg {
	public:


		SlaveMessageOutput stctOutput;
		SlaveMessageInput stctInput;
		unsigned int jointNumber;

		// Constructor

		YouBotSlaveMsg() {
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
  
  ///////////////////////////////////////////////////////////////////////////////
	/// EtherCat message of the youBot EtherCat slaves which is thread safe
  ///////////////////////////////////////////////////////////////////////////////
	class YouBotSlaveMsgThreadSafe {
	public:


		DataObjectLockFree<SlaveMessageOutput> stctOutput;
		DataObjectLockFree<SlaveMessageInput> stctInput;
		DataObjectLockFree<unsigned int> jointNumber;

		// Constructor
		YouBotSlaveMsgThreadSafe() {
			jointNumber.Set(0);
		}

    
		// Copy-Constructor
		YouBotSlaveMsgThreadSafe(const YouBotSlaveMsgThreadSafe &copy) {
      SlaveMessageOutput tempOutput;
      SlaveMessageInput tempInput;
      unsigned int tempjointNo;
      
      copy.stctOutput.Get(tempOutput);
			stctOutput.Set(tempOutput);
      
      copy.stctInput.Get(tempInput);
			stctInput.Set(tempInput);
              
      copy.jointNumber.Get(tempjointNo);
			jointNumber.Set(tempjointNo);
		}

		// Destructor
		~YouBotSlaveMsgThreadSafe() {
		}

		// assignment operator
		YouBotSlaveMsgThreadSafe & operator=(const YouBotSlaveMsgThreadSafe &copy) {
			SlaveMessageOutput tempOutput;
      SlaveMessageInput tempInput;
      unsigned int tempjointNo;
      
      copy.stctOutput.Get(tempOutput);
			stctOutput.Set(tempOutput);
      
      copy.stctInput.Get(tempInput);
			stctInput.Set(tempInput);
              
      copy.jointNumber.Get(tempjointNo);
			jointNumber.Set(tempjointNo);

			return *this;
		}
 
	};

} // namespace youbot

#endif	/* _YOUBOT_SLAVE_MESSAGE_H */
