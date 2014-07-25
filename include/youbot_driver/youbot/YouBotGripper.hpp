#ifndef YOUBOT_YOUBOTGRIPPER_H
#define YOUBOT_YOUBOTGRIPPER_H

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
#include "youbot_driver/generic/Units.hpp"
#include "youbot_driver/generic/Time.hpp"
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
#include "youbot_driver/youbot/YouBotGripperBar.hpp"
#include <boost/scoped_ptr.hpp>

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// The youBot gripper with one degree of freedom
///////////////////////////////////////////////////////////////////////////////
class YouBotGripper : public OneDOFGripper {
  public:
    YouBotGripper(const unsigned int jointNo, const std::string& configFilePath = "../config/");

    virtual ~YouBotGripper();


  protected:
    virtual void getConfigurationParameter(GripperParameter& parameter) const;

    virtual void setConfigurationParameter(const GripperParameter& parameter);


  public:
    virtual void getConfigurationParameter(GripperFirmwareVersion& parameter) const;

    virtual void setConfigurationParameter(const CalibrateGripper& parameter);

    virtual void getConfigurationParameter(YouBotSlaveMailboxMsg& parameter) const;


  protected:
    virtual void getData(const GripperData& data) const;

    virtual void setData(const GripperData& data);

    virtual void getData(OneDOFGripperData& data) const;

    virtual void setData(const OneDOFGripperData& data);


  public:
    virtual void setData(const GripperBarSpacingSetPoint& barSpacing);

    virtual void getData(GripperSensedBarSpacing& barSpacing) const;

    void open();

    void close();

    YouBotGripperBar& getGripperBar1();

    YouBotGripperBar& getGripperBar2();


  private:
    YouBotGripper(const YouBotGripper & source);

    YouBotGripper & operator=(const YouBotGripper & source);

    void parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) const;

    bool setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg) const;

    bool retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) const;

    EthercatMasterInterface* ethercatMaster;

    unsigned int timeTillNextMailboxUpdate;

    unsigned int mailboxMsgRetries;

    unsigned int jointNumber;

    boost::scoped_ptr<YouBotGripperBar> bar1;

    boost::scoped_ptr<YouBotGripperBar> bar2;

};

} // namespace youbot
#endif
