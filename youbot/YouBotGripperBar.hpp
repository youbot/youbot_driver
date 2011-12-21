#ifndef YOUBOT_YOUBOTGRIPPERBAR_H
#define YOUBOT_YOUBOTGRIPPERBAR_H

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
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/Exceptions.hpp"
#include "youbot/ProtocolDefinitions.hpp"

#ifdef ETHERCAT_MASTER_WITHOUT_THREAD
  #include "youbot/EthercatMasterWithoutThread.hpp"
#else
  #include "youbot/EthercatMaster.hpp"
#endif

#include "youbot/YouBotSlaveMsg.hpp"
#include "youbot/YouBotSlaveMailboxMsg.hpp"
#include "generic-gripper/Gripper.hpp"
#include "generic-gripper/GripperData.hpp"
#include "generic-gripper/GripperParameter.hpp"
#include "one-dof-gripper/OneDOFGripper.hpp"
#include "one-dof-gripper/OneDOFGripperData.hpp"
#include "youbot/YouBotGripperParameter.hpp"

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// One bar of the youBot gripper
///////////////////////////////////////////////////////////////////////////////
class YouBotGripperBar {
  public:
    YouBotGripperBar(const unsigned int barNo, const unsigned int jointNo, const std::string& configFilePath = "../config/");

    virtual ~YouBotGripperBar();

    virtual void getConfigurationParameter(YouBotGripperParameter& parameter);

    virtual void setConfigurationParameter(const YouBotGripperParameter& parameter);

    virtual void getConfigurationParameter(YouBotSlaveMailboxMsg& parameter);

    virtual void setData(const GripperBarEncoterSetpoint& encoderSetpoint);

    virtual void getData(GripperSensedVelocity& barVelocity);


  private:
    void parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg);

    bool setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg);

    bool retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message);

    EthercatMaster* ethercatMaster;

    unsigned int timeTillNextMailboxUpdate;

    unsigned int mailboxMsgRetries;

    unsigned int jointNumber;

    unsigned int barNo;

};

} // namespace youbot
#endif
