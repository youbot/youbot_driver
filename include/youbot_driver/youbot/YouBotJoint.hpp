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
#include <boost/scoped_ptr.hpp>
#include "youbot_driver/generic-joint/Joint.hpp"
#include "youbot_driver/generic-joint/JointData.hpp"
#include "youbot_driver/youbot/YouBotJointStorage.hpp"
#include "youbot_driver/youbot/ProtocolDefinitions.hpp"
#include "youbot_driver/youbot/YouBotJointParameter.hpp"
#include "youbot_driver/youbot/YouBotJointParameterPasswordProtected.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot_driver/youbot/EthercatMasterInterface.hpp"
#include "youbot_driver/youbot/JointTrajectoryController.hpp"
#include "youbot_driver/youbot/JointLimitMonitor.hpp"

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// youBot joint for the base and the manipulator.
/// Every motor, encoder, transmition combination of the youBot base or manipulator is a YouBotJoint
///////////////////////////////////////////////////////////////////////////////
class YouBotJoint : public Joint {
  public:
    YouBotJoint(const unsigned int jointNo, const std::string& configFilePath = "../config/");

    ~YouBotJoint();


  private:
    YouBotJoint(const YouBotJoint & source);

    YouBotJoint & operator=(const YouBotJoint & source);


  protected:
    virtual void setConfigurationParameter(const JointParameter& parameter);

    virtual void getConfigurationParameter(JointParameter& parameter);


  public:
    virtual void getConfigurationParameter(YouBotJointParameterReadOnly& parameter);

    virtual void getConfigurationParameter(YouBotJointParameter& parameter);

    virtual void setConfigurationParameter(const YouBotJointParameter& parameter);

    virtual void getConfigurationParameter(JointName& parameter);

    virtual void setConfigurationParameter(const JointName& parameter);

    virtual void getConfigurationParameter(GearRatio& parameter);

    virtual void setConfigurationParameter(const GearRatio& parameter);

    virtual void getConfigurationParameter(EncoderTicksPerRound& parameter);

    virtual void setConfigurationParameter(const EncoderTicksPerRound& parameter);

    virtual void setConfigurationParameter(const CalibrateJoint& parameter);

    virtual void setConfigurationParameter(const InverseMovementDirection& parameter);

    virtual void getConfigurationParameter(InverseMovementDirection& parameter);

    virtual void setConfigurationParameter(const JointLimits& parameter);

    virtual void getConfigurationParameter(JointLimits& parameter);

    virtual void getConfigurationParameter(JointLimitsRadian& parameter);

    /// commutation method for firmware 1.48 and below
    virtual void setConfigurationParameter(const InitializeJoint& parameter);

    virtual void getConfigurationParameter(FirmwareVersion& parameter);

    ///this method should be only used if you know what you are doing
    virtual void setConfigurationParameter(const YouBotSlaveMailboxMsg& parameter);

    ///this method should be only used if you know what you are doing
    virtual void getConfigurationParameter(YouBotSlaveMailboxMsg& parameter);

    virtual void getConfigurationParameter(TorqueConstant& parameter);

    virtual void setConfigurationParameter(const TorqueConstant& parameter);

    ///stores the joint parameter permanent in the EEPROM of the motor contoller
    ///Attentions: The EEPROM has only a finite number of program-erase cycles
    void storeConfigurationParameterPermanent(const YouBotJointParameter& parameter);

    /// Restores the joint parameter from the EEPROM.
    void restoreConfigurationParameter(YouBotJointParameter& parameter);


  protected:
    virtual void setData(const JointDataSetpoint& data);

    virtual void getData(JointData& data);


  public:
    ///commands a position or angle to one joint
    ///@param data the to command position
    virtual void setData(const JointAngleSetpoint& data);

    ///commands a encoder value (position) to one joint
    ///@param data the to command encoder value
    virtual void setData(const JointEncoderSetpoint& data);

    ///gets the position or angle of one joint which have been calculated from the actual encoder value 
    ///@param data returns the angle by reference
    virtual void getData(JointSensedAngle& data);

    ///commands a velocity to one joint
    ///@param data the to command velocity
    virtual void setData(const JointVelocitySetpoint& data);

    ///gets the velocity of one joint
    ///@param data returns the velocity by reference
    virtual void getData(JointSensedVelocity& data);

    ///gets the velocity in round per minute of one joint
    ///@param data returns the velocity by reference
    virtual void getData(JointSensedRoundsPerMinute& data);

    ///sets the velocity in round per minute to one joint
    ///@param data the setpoint velocity
    virtual void setData(const JointRoundsPerMinuteSetpoint& data);

    ///gets the motor current of one joint which have been measured by a hal sensor
    ///@param data returns the actual motor current by reference
    virtual void getData(JointSensedCurrent& data);

    ///commands a current to one joint
    ///@param data the to command current
    virtual void setData(const JointCurrentSetpoint& data);

    ///gets the encoder ticks of one joint
    ///@param data returns the ticks by reference
    virtual void getData(JointSensedEncoderTicks& data);

    ///sets the output part of a EtherCAT slave message
    ///this methode should be only used if you know what you are doing
    ///@param data output part of a EtherCAT slave message
    virtual void setData(const SlaveMessageOutput& data);

    ///gets the input and ouput part of a EtherCAT slave message
    ///this methode should be only used if you know what you are doing
    ///@param data returns the sensor values by reference
    virtual void getData(YouBotSlaveMsg& data);

    ///commands a torque to one joint
    ///@param data the to command torque
    virtual void setData(const JointTorqueSetpoint& data);

    ///gets the motor torque of one joint which have been calculated from the current
    ///@param data returns the actual motor torque by reference
    virtual void getData(JointSensedTorque& data);

    ///gets the target or setpoint position of one joint (only firmware 2.0 or higher)
    ///@param data returns the angle by reference
    virtual void getData(JointAngleSetpoint& data);

    ///gets the target or setpoint velocity of one joint (only firmware 2.0 or higher)
    ///@param data returns the velocity by reference
    virtual void getData(JointVelocitySetpoint& data);

    ///gets the motor current target or setpoint of one joint (only firmware 2.0 or higher)
    ///@param data returns the motor current by reference
    virtual void getData(JointCurrentSetpoint& data);

    ///gets the ramp generator velocity of one joint (only firmware 2.0 or higher)
    ///@param data returns the velocity by reference
    virtual void getData(JointRampGeneratorVelocity& data);

    void getUserVariable(const unsigned int index, int& data);

    void setUserVariable(const unsigned int index, const int data);

    /// Returns the status messages for the motor controller. 
    virtual void getStatus(std::vector<std::string>& statusMessages);

    /// Returns the status messages as status flags for the motor controller. The status flag bits are assigned like this:
    /// 0:  Overcurrent
    /// 1:  Undervoltage
    /// 2:  Overvoltage
    /// 3:  Overtemperature
    /// 4:  Motor halted
    /// 5:  Hall error flag
    /// 6:  ---
    /// 7:  ---
    /// 8:  PWM mode active
    /// 9:  Velocity mode active
    /// 10: Position mode active
    /// 11: Torque mode active
    /// 12: ---
    /// 13: ---
    /// 14: Position end flag
    /// 15: Module initialized
    /// 16: EtherCAT timeout flag
    /// 17: I2t exceeded flag
    virtual void getStatus(unsigned int& statusFlags);

    /// set the encoder values of the joint to zero. This postion will be the new reference.
    void setEncoderToZero();

    void noMoreAction();

    void stopJoint();

    unsigned int getJointNumber();


  private:
    void parseYouBotErrorFlags(const YouBotSlaveMsg& messageBuffer);

    void parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg);

    bool retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message);

    bool setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg);


  public:
    JointTrajectoryController trajectoryController;


  private:
    EthercatMasterInterface* ethercatMaster;

    bool positionReferenceToZero;

    unsigned int timeTillNextMailboxUpdate;

    unsigned int mailboxMsgRetries;

    YouBotJointStorage storage;

    YouBotSlaveMsg messageBuffer;

    quantity<si::angular_velocity> lastVelocity;

    quantity<plane_angle> lastPosition;

    boost::scoped_ptr<JointLimitMonitor> limitMonitor;

};

} // namespace youbot
#endif
