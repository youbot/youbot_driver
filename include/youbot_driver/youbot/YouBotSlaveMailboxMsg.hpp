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

#include <youbot_driver/soem/ethercattype.h>
#include <string>
#include <time.h>
#include "youbot_driver/generic/dataobjectlockfree/DataObjectLockFree.hpp"

namespace youbot {

  /// Output part from the EtherCAT mailbox message of the youBot slaves

  PACKED_BEGIN
  struct mailboxOutputBuffer {
    uint8 moduleAddress; //0 = Drive  1 = Gripper
    uint8 commandNumber;
    uint8 typeNumber;
    uint8 motorNumber; //always zero
    uint32 value; //MSB first!

    mailboxOutputBuffer() : moduleAddress(0), commandNumber(0), typeNumber(0), motorNumber(0), value(0) {};
  } PACKED;
  PACKED_END

  /// Input part from the EtherCAT mailbox message of the youBot slaves

  PACKED_BEGIN
  struct mailboxInputBuffer {
    uint8 replyAddress;
    uint8 moduleAddress;
    uint8 status; //(e.g. 100 means “no error”)
    uint8 commandNumber;
    uint32 value; //MSB first!

    mailboxInputBuffer() : replyAddress(0), moduleAddress(0), status(0), commandNumber(0), value(0) {};
  } PACKED;
  PACKED_END

  ///////////////////////////////////////////////////////////////////////////////
  /// EtherCAT mailbox message of the youBot slaves 
  ///////////////////////////////////////////////////////////////////////////////

 class YouBotSlaveMailboxMsg {
  public:

    mailboxOutputBuffer stctOutput;
    mailboxInputBuffer stctInput;

    // Constructor
    YouBotSlaveMailboxMsg() {
      slaveNumber = 1000;
    }

    // Constructor

    YouBotSlaveMailboxMsg(unsigned int slaveNo) {
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
    unsigned int slaveNumber;
  };
  
  
  ///////////////////////////////////////////////////////////////////////////////
  /// EtherCAT mailbox message of the youBot slaves (thread safe)
  ///////////////////////////////////////////////////////////////////////////////
  class YouBotSlaveMailboxMsgThreadSafe {
  public:

    DataObjectLockFree<mailboxOutputBuffer> stctOutput;
    DataObjectLockFree<mailboxInputBuffer> stctInput;

    // Constructor
    YouBotSlaveMailboxMsgThreadSafe() {
      slaveNumber.Set(1000);
    }

    // Constructor

    YouBotSlaveMailboxMsgThreadSafe(unsigned int slaveNo) {
      slaveNumber.Set(slaveNo);
    }
    // Copy-Constructor

    YouBotSlaveMailboxMsgThreadSafe(const YouBotSlaveMailboxMsgThreadSafe& copy) {
      mailboxOutputBuffer tempStctOutput;
      mailboxInputBuffer tempStctInput;
      std::string tempParameterName;
      unsigned int SlaveNumber;
      
      
      copy.stctOutput.Get(tempStctOutput);
      stctOutput.Set(tempStctOutput);
      
      copy.stctInput.Get(tempStctInput);
      stctInput.Set(tempStctInput);
      
      copy.slaveNumber.Get(SlaveNumber);
      slaveNumber.Set(SlaveNumber);
      
      copy.parameterName.Get(tempParameterName);
      parameterName.Set(tempParameterName);
    }

    // Destructor

    ~YouBotSlaveMailboxMsgThreadSafe() {
    }

    // assignment operator

    YouBotSlaveMailboxMsgThreadSafe & operator=(const YouBotSlaveMailboxMsgThreadSafe& copy) {
      mailboxOutputBuffer tempStctOutput;
      mailboxInputBuffer tempStctInput;
      std::string tempParameterName;
      unsigned int SlaveNumber;
      
      
      copy.stctOutput.Get(tempStctOutput);
      stctOutput.Set(tempStctOutput);
      
      copy.stctInput.Get(tempStctInput);
      stctInput.Set(tempStctInput);
      
      copy.slaveNumber.Get(SlaveNumber);
      slaveNumber.Set(SlaveNumber);
      
      copy.parameterName.Get(tempParameterName);
      parameterName.Set(tempParameterName);
      return *this;
    }

    DataObjectLockFree<std::string> parameterName;

    DataObjectLockFree<unsigned int> slaveNumber;
  };
  
 

} // namespace youbot

#endif	/* _YOUBOT_SLAVE_MESSAGE_H */
