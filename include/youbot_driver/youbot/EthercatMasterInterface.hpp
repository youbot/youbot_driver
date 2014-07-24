#ifndef YOUBOT_ETHERCATMASTERINTERFACE_H
#define YOUBOT_ETHERCATMASTERINTERFACE_H

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
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot_driver/youbot/JointLimitMonitor.hpp"
extern "C"{
#include <youbot_driver/soem/ethercattype.h>
#include <nicdrv.h>
#include <youbot_driver/soem/ethercatmain.h>
}

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// The Ethercat Master interface
///////////////////////////////////////////////////////////////////////////////
class EthercatMasterInterface {
friend class EthercatMaster;
friend class YouBotJoint;
friend class YouBotGripper;
friend class YouBotGripperBar;
  protected:
    EthercatMasterInterface() {};

    virtual ~EthercatMasterInterface() {};


  public:
    virtual bool isThreadActive() = 0;

    ///return the quantity of ethercat slave which have an input/output buffer
    virtual unsigned int getNumberOfSlaves() const = 0;

    virtual void AutomaticSendOn(const bool enableAutomaticSend) = 0;

    virtual void AutomaticReceiveOn(const bool enableAutomaticReceive) = 0;

    ///provides all ethercat slave informations from the SOEM driver
    ///@param ethercatSlaveInfos ethercat slave informations
    virtual void getEthercatDiagnosticInformation(std::vector<ec_slavet>& ethercatSlaveInfos) = 0;

    ///sends ethercat messages to the motor controllers
    /// returns a true if everything it OK and returns false if something fail
    virtual bool sendProcessData() = 0;

    /// receives ethercat messages from the motor controllers
    /// returns a true if everything it OK and returns false if something fail
    virtual bool receiveProcessData() = 0;

    /// checks if an error has occurred in the soem driver
    /// returns a true if an error has occurred
    virtual bool isErrorInSoemDriver() = 0;

    virtual bool isEtherCATConnectionEstablished() = 0;

    virtual void registerJointLimitMonitor(JointLimitMonitor* object, const unsigned int JointNumber) = 0;


  private:
    ///stores a ethercat message to the buffer
    ///@param msgBuffer ethercat message
    ///@param jointNumber joint number of the sender joint
    virtual void setMsgBuffer(const YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber) = 0;

    ///get a ethercat message form the buffer
    ///@param msgBuffer ethercat message
    ///@param jointNumber joint number of the receiver joint
    virtual void getMsgBuffer(const unsigned int jointNumber, YouBotSlaveMsg& returnMsg) = 0;

    ///stores a mailbox message in a buffer which will be sent to the motor controllers
    ///@param msgBuffer ethercat mailbox message
    ///@param jointNumber joint number of the sender joint
    virtual void setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber) = 0;

    ///gets a mailbox message form the buffer which came form the motor controllers
    ///@param msgBuffer ethercat mailbox message
    ///@param jointNumber joint number of the receiver joint
    virtual bool getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber) = 0;

};

} // namespace youbot
#endif
