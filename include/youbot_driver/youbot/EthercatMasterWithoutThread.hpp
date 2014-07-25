#ifndef YOUBOT_ETHERCATMASTERWITHOUTTHREAD_H
#define YOUBOT_ETHERCATMASTERWITHOUTTHREAD_H

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

#include <vector>
#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "youbot_driver/generic/Units.hpp"
#include "youbot_driver/generic/Time.hpp"
#include "youbot_driver/generic/ConfigFile.hpp"
#include "youbot_driver/youbot/ProtocolDefinitions.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"
#include "youbot_driver/youbot/EthercatMasterInterface.hpp"
#include "youbot_driver/youbot/JointLimitMonitor.hpp"

extern "C"{
#include <youbot_driver/soem/ethercattype.h>
#include <youbot_driver/soem/ethercatmain.h>
}

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// The Ethercat Master is managing the whole ethercat communication 
/// It have to be a singleton in the system
///////////////////////////////////////////////////////////////////////////////
class EthercatMasterWithoutThread : public EthercatMasterInterface {
friend class EthercatMaster;
friend class YouBotJoint;
friend class YouBotGripper;
friend class YouBotGripperBar;
  private:
    EthercatMasterWithoutThread(const std::string& configFile, const std::string& configFilePath);

    ~EthercatMasterWithoutThread();


  public:
    bool isThreadActive();

    ///return the quantity of ethercat slave which have an input/output buffer
    unsigned int getNumberOfSlaves() const;

    void AutomaticSendOn(const bool enableAutomaticSend);

    void AutomaticReceiveOn(const bool enableAutomaticReceive);

    ///provides all ethercat slave informations from the SOEM driver
    ///@param ethercatSlaveInfos ethercat slave informations
    void getEthercatDiagnosticInformation(std::vector<ec_slavet>& ethercatSlaveInfos);

    ///sends ethercat messages to the motor controllers
    /// returns a true if everything it OK and returns false if something fail
    bool sendProcessData();

    /// receives ethercat messages from the motor controllers
    /// returns a true if everything it OK and returns false if something fail
    bool receiveProcessData();

    /// checks if an error has occurred in the soem driver
    /// returns a true if an error has occurred
    bool isErrorInSoemDriver();

    bool isEtherCATConnectionEstablished();

    void registerJointLimitMonitor(JointLimitMonitor* object, const unsigned int JointNumber);


  private:
    ///establishes the ethercat connection
    void initializeEthercat();

    ///closes the ethercat connection
    bool closeEthercat();

    ///stores a ethercat message to the buffer
    ///@param msgBuffer ethercat message
    ///@param jointNumber joint number of the sender joint
    void setMsgBuffer(const YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber);

    ///get a ethercat message form the buffer
    ///@param msgBuffer ethercat message
    ///@param jointNumber joint number of the receiver joint
    void getMsgBuffer(const unsigned int jointNumber, YouBotSlaveMsg& returnMsg);

    ///stores a mailbox message in a buffer which will be sent to the motor controllers
    ///@param msgBuffer ethercat mailbox message
    ///@param jointNumber joint number of the sender joint
    void setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber);

    ///gets a mailbox message form the buffer which came form the motor controllers
    ///@param msgBuffer ethercat mailbox message
    ///@param jointNumber joint number of the receiver joint
    bool getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber);

    ///sends the mailbox Messages which have been stored in the buffer
    ///@param mailboxMsg ethercat mailbox message
    bool sendMailboxMessage(const YouBotSlaveMailboxMsg& mailboxMsg);

    ///receives mailbox messages and stores them in the buffer
    ///@param mailboxMsg ethercat mailbox message
    bool receiveMailboxMessage(YouBotSlaveMailboxMsg& mailboxMsg);

    void parseYouBotErrorFlags(const YouBotSlaveMsg& messageBuffer);

    std::string ethernetDevice;

    std::vector<YouBotSlaveMsg> processDataBuffer;

    unsigned int nrOfSlaves;

    std::vector<SlaveMessageOutput*> ethercatOutputBufferVector;

    std::vector<SlaveMessageInput*> ethercatInputBufferVector;

    std::vector<YouBotSlaveMailboxMsg> firstMailboxBufferVector;

    ec_mbxbuft mailboxBufferSend;

    unsigned int mailboxTimeout;

    ec_mbxbuft mailboxBufferReceive;

    ConfigFile* configfile;

    std::vector<ec_slavet> ethercatSlaveInfo;

    char IOmap_[4096];

    unsigned int ethercatTimeout;

    static std::string configFileName;

    static std::string configFilepath;

    bool ethercatConnectionEstablished;

};

} // namespace youbot
#endif
