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
#include "youbot_driver/generic/Logger.hpp"
#include "youbot_driver/generic/Units.hpp"
#include "youbot_driver/generic/Time.hpp"
#include "youbot_driver/generic/Exceptions.hpp"
#include "youbot_driver/youbot/ProtocolDefinitions.hpp"
#include "youbot_driver/youbot/EthercatMasterInterface.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot_driver/generic-gripper/Gripper.hpp"
#include "youbot_driver/generic-gripper/GripperData.hpp"
#include "youbot_driver/generic-gripper/GripperParameter.hpp"
#include "youbot_driver/one-dof-gripper/OneDOFGripper.hpp"
#include "youbot_driver/one-dof-gripper/OneDOFGripperData.hpp"
#include "youbot_driver/youbot/YouBotGripperParameter.hpp"

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// One bar of the youBot gripper
///////////////////////////////////////////////////////////////////////////////
class YouBotGripperBar {
  public:
    YouBotGripperBar(const unsigned int barNo, const unsigned int jointNo, const std::string& configFilePath = "../config/");

    virtual ~YouBotGripperBar();

    virtual void setConfigurationParameter(const MaxEncoderValue& parameter);

    virtual void getConfigurationParameter(MaxEncoderValue& parameter) const;

    virtual void getConfigurationParameter(MaxTravelDistance& parameter) const;

    virtual void setConfigurationParameter(const MaxTravelDistance& parameter);

    virtual void setConfigurationParameter(const BarSpacingOffset& parameter);

    virtual void getConfigurationParameter(BarSpacingOffset& parameter) const;

    virtual void setConfigurationParameter(const GripperBarName& parameter);

    virtual void getConfigurationParameter(GripperBarName& parameter) const;

    virtual void getConfigurationParameter(YouBotGripperParameter& parameter) const;

    virtual void setConfigurationParameter(const YouBotGripperParameter& parameter);

    virtual void getConfigurationParameter(YouBotSlaveMailboxMsg& parameter) const;

    virtual void setData(const GripperBarEncoderSetpoint& encoderSetpoint);

    virtual void getData(GripperSensedVelocity& barVelocity) const;

    virtual void getData(GripperSensedBarPosition& barPosition) const;

    virtual void setData(GripperBarPositionSetPoint& barPosition);

    void parseGripperErrorFlags(const unsigned int& errosFlags);


  private:
    YouBotGripperBar(const YouBotGripperBar & source);

    YouBotGripperBar & operator=(const YouBotGripperBar & source);

    void parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) const;

    bool setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg) const;

    bool retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) const;

    quantity<si::length> maxTravelDistance;

    unsigned int maxEncoderValue;

    quantity<si::length> barSpacingOffset;

    EthercatMasterInterface* ethercatMaster;

    unsigned int timeTillNextMailboxUpdate;

    unsigned int mailboxMsgRetries;

    unsigned int jointNumber;

    unsigned int barNo;

    std::string name;

};

} // namespace youbot
#endif
