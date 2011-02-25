#ifndef YOUBOT_YOUBOTJOINT_H
#define YOUBOT_YOUBOTJOINT_H

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
#include <cmath>
#include <boost/thread.hpp>
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/Exceptions.hpp"
#include "generic-joint/Joint.hpp"
#include "generic-joint/JointData.hpp"
#include "youbot/YouBotJointStorage.hpp"
#include "youbot/ProtocolDefinitions.hpp"
#include "youbot/YouBotJointParameter.hpp"
#include "youbot/YouBotSlaveMsg.hpp"
#include "youbot/YouBotSlaveMailboxMsg.hpp"

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// youBot joint for the base and the manipulator.
/// Every motor, encoder, transmition combination of the youBot base or manipulator is a YouBotJoint
///////////////////////////////////////////////////////////////////////////////
class YouBotJoint : public Joint {
  public:
    YouBotJoint(unsigned int jointNo);

    ~YouBotJoint();


  protected:
    virtual void setConfigurationParameter(const JointParameter& parameter);

    virtual void getConfigurationParameter(JointParameter& parameter);


  public:
    void getConfigurationParameter(YouBotJointParameterReadOnly& parameter);

    void getConfigurationParameter(YouBotJointParameter& parameter);

    void setConfigurationParameter(const YouBotJointParameter& parameter);

    void getConfigurationParameter(JointName& parameter);

    void setConfigurationParameter(const JointName& parameter);

    void getConfigurationParameter(GearRatio& parameter);

    void setConfigurationParameter(const GearRatio& parameter);

    void getConfigurationParameter(EncoderTicksPerRound& parameter);

    void setConfigurationParameter(const EncoderTicksPerRound& parameter);

    void setConfigurationParameter(const CalibrateJoint& parameter);

    void setConfigurationParameter(const InverseMovementDirection& parameter);

    void setConfigurationParameter(const JointLimits& parameter);

    void setConfigurationParameter(const StopJoint& parameter);

    void setConfigurationParameter(const NoMoreAction& parameter);

    ///stores the joint parameter permanent in the EEPROM of the motor contoller
    ///Attentions: The EEPROM has only a finite number of program-erase cycles
    void storeConfigurationParameterPermanent(const YouBotJointParameter& parameter);

    /// Restores the joint parameter from the EEPROM.
    void restoreConfigurationParameter(YouBotJointParameter& parameter);


  protected:
    virtual void setData(const JointDataSetpoint& data, SyncMode communicationMode = NON_BLOCKING);

    virtual void getData(JointData& data);


  public:
    ///commands a position or angle to one joint
    ///@param data the to command position
    ///@param communicationMode at the moment only non blocking communication is implemented
    virtual void setData(const JointAngleSetpoint& data, SyncMode communicationMode = NON_BLOCKING);

    ///gets the position or angle of one joint which have been calculated from the actual encoder value 
    ///@param data returns the angle by reference
    virtual void getData(JointSensedAngle& data);

    ///commands a velocity to one joint
    ///@param data the to command velocity
    ///@param communicationMode at the moment only non blocking communication is implemented
    virtual void setData(const JointVelocitySetpoint& data, SyncMode communicationMode = NON_BLOCKING);

    ///gets the velocity of one joint which have been calculated from the actual encoder values
    ///@param data returns the velocity by reference
    virtual void getData(JointSensedVelocity& data);

    ///gets temperature of the motor which have been measured by a thermometer
    ///@param data returns the actual temperature by reference
    virtual void getData(JointSensedTemperature& data);

    ///gets the motor current of one joint which have been measured by a hal sensor
    ///@param data returns the actual motor current by reference
    virtual void getData(JointSensedCurrent& data);


  private:
    void parseYouBotErrorFlags(const YouBotSlaveMsg& messageBuffer);

    void parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg);

    bool retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message);

    bool setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg);

    std::string jointName;

    unsigned int jointNumber;

    bool positionReferenceToZero;

    unsigned int timeTillNextMailboxUpdate;

    unsigned int mailboxMsgRetries;

    YouBotJointStorage storage;

};

} // namespace youbot
#endif
