#ifndef YOUBOT_YOUBOTJOINTPARAMETERPASSWORDPROTECTED_H
#define YOUBOT_YOUBOTJOINTPARAMETERPASSWORDPROTECTED_H

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
#include <boost/limits.hpp>
#include "youbot_driver/generic/Logger.hpp"
#include "youbot_driver/generic/Units.hpp"
#include "youbot_driver/generic/Time.hpp"
#include "youbot_driver/generic/Exceptions.hpp"
#include "youbot_driver/generic-joint/JointParameter.hpp"
#include "youbot_driver/youbot/YouBotJointParameter.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot_driver/youbot/YouBotJointStorage.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract youBot joint parameter
///////////////////////////////////////////////////////////////////////////////
class YouBotJointParameterPasswordProtected : public YouBotJointParameter {
friend class YouBotJoint;
  protected:
    YouBotJointParameterPasswordProtected();


  public:
    virtual ~YouBotJointParameterPasswordProtected();

    virtual void toString(std::string& value) = 0;


  protected:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const = 0;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) = 0;

    virtual std::string getName() const = 0;

    virtual ParameterType getType() const = 0;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Enable overvoltage protection. 
///////////////////////////////////////////////////////////////////////////////
class ActivateOvervoltageProtection : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    ActivateOvervoltageProtection();

    virtual ~ActivateOvervoltageProtection();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// This value represents the internal commutation offset. (0 ... max. Encoder steps per rotation)
///////////////////////////////////////////////////////////////////////////////
class ActualCommutationOffset : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    ActualCommutationOffset();

    virtual ~ActualCommutationOffset();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Enter a password to approve the change of protected parameters.
///////////////////////////////////////////////////////////////////////////////
class ApproveProtectedParameters : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    ApproveProtectedParameters();

    virtual ~ApproveProtectedParameters();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// BEMF constant of motor. Used for current regulation, position regulation and velocity regulation. Feed forward control for current regulation, position regulation and velocity regulation is disabled if BEMF constant is set to zero.
///////////////////////////////////////////////////////////////////////////////
class BEMFConstant : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    BEMFConstant();

    virtual ~BEMFConstant();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The Commutation Mode.
/// 0: Block commutation with hall sensors mode \n
/// 1: Sensorless block commutation (hallFX) \n
/// 2: Sine commutation with hall sensors \n
/// 3: Sine commutation with encoder \n
/// 4: Controlled block commutation \n
/// 5: Controlled sine commutation \n
///////////////////////////////////////////////////////////////////////////////
class CommutationMode : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    CommutationMode();

    virtual ~CommutationMode();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Motor current for controlled commutation. This parameter is used in commutation mode 1, 4, 5 and in initialization of sine.
///////////////////////////////////////////////////////////////////////////////
class CommutationMotorCurrent : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    CommutationMotorCurrent();

    virtual ~CommutationMotorCurrent();

    void getParameter(quantity<current>& parameter) const;

    void setParameter(const quantity<current>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<current> upperLimit;

    quantity<current> lowerLimit;

    quantity<current> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Delay of current limitation algorithm / PID current regulator. 
///////////////////////////////////////////////////////////////////////////////
class CurrentControlLoopDelay : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    CurrentControlLoopDelay();

    virtual ~CurrentControlLoopDelay();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::time> upperLimit;

    quantity<si::time> lowerLimit;

    quantity<si::time> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Encoder Steps per Rotation.
///////////////////////////////////////////////////////////////////////////////
class EncoderResolution : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    EncoderResolution();

    virtual ~EncoderResolution();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Encoder stop switch.
/// Bit 0: Left stop switch enable \n
/// When this bit is set the motor will be stopped if it is moving in negative direction and the left stop switch input becomes active.\n\n
/// Bit 1: Right stop switch enable \n
/// When this bit is set the motor will be stopped if it is moving in positive direction and the right stop switch input becomes active\n\n
/// Please see StopSwitchPolarity for selecting the stop switch input polarity.
///////////////////////////////////////////////////////////////////////////////
class EncoderStopSwitch : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    EncoderStopSwitch();

    virtual ~EncoderStopSwitch();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Hall sensor invert. Sets one of the motors invert with inverted hall scheme, e.g. some Maxon motors
///////////////////////////////////////////////////////////////////////////////
class HallSensorPolarityReversal : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    HallSensorPolarityReversal();

    virtual ~HallSensorPolarityReversal();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Counts how often an I2t sum was higher than the I2t limit.
///////////////////////////////////////////////////////////////////////////////
class I2tExceedCounter : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    I2tExceedCounter();

    virtual ~I2tExceedCounter();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// An actual I2t sum that exceeds this limit leads to increasing the I2t exceed counter.
///////////////////////////////////////////////////////////////////////////////
class I2tLimit : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    I2tLimit();

    virtual ~I2tLimit();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Initialization Mode.
/// 0: Initialization in controlled sine commutation \n
/// 1: Initialization in block commutation by using hall sensors \n 
///////////////////////////////////////////////////////////////////////////////
class InitializationMode : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    InitializationMode();

    virtual ~InitializationMode();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Duration for sine initialization sequence. This parameter should be set in a way, that the motor has stopped mechanical oscillations after the specified time. 
///////////////////////////////////////////////////////////////////////////////
class InitSineDelay : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    InitSineDelay();

    virtual ~InitSineDelay();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::time> upperLimit;

    quantity<si::time> lowerLimit;

    quantity<si::time> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Mass inertia constant for position regulation. Compensates mass moment of inertia of rotor.
///////////////////////////////////////////////////////////////////////////////
class MassInertiaConstant : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    MassInertiaConstant();

    virtual ~MassInertiaConstant();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// maximum allowed current
///////////////////////////////////////////////////////////////////////////////
class MaximumMotorCurrent : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    MaximumMotorCurrent();

    virtual ~MaximumMotorCurrent();

    void getParameter(quantity<current>& parameter) const;

    void setParameter(const quantity<current>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<current> upperLimit;

    quantity<current> lowerLimit;

    quantity<current> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Resistance of motor coil. Used for current resistance regulation, position regulation and velocity regulation.
///////////////////////////////////////////////////////////////////////////////
class MotorCoilResistance : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    MotorCoilResistance();

    virtual ~MotorCoilResistance();

    void getParameter(quantity<resistance>& parameter) const;

    void setParameter(const quantity<resistance>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<resistance> upperLimit;

    quantity<resistance> lowerLimit;

    quantity<resistance> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Set/Get Timeout to determine an interrupted communication with the EtherCAT master. (automatically stored in EEProm)
///////////////////////////////////////////////////////////////////////////////
class MotorControllerTimeout : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    MotorControllerTimeout();

    virtual ~MotorControllerTimeout();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::time> upperLimit;

    quantity<si::time> lowerLimit;

    quantity<si::time> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Number of motor poles.
///////////////////////////////////////////////////////////////////////////////
class MotorPoles : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    MotorPoles();

    virtual ~MotorPoles();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Counts the module operational time.
///////////////////////////////////////////////////////////////////////////////
class OperationalTime : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    OperationalTime();

    virtual ~OperationalTime();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::time> upperLimit;

    quantity<si::time> lowerLimit;

    quantity<si::time> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// PID calculation delay: Set operational frequency PID
///////////////////////////////////////////////////////////////////////////////
class PIDControlTime : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    PIDControlTime();

    virtual ~PIDControlTime();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::time> upperLimit;

    quantity<si::time> lowerLimit;

    quantity<si::time> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Encoder direction Set this flag in a way, that turn right increases position counter.
///////////////////////////////////////////////////////////////////////////////
class ReversingEncoderDirection : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    ReversingEncoderDirection();

    virtual ~ReversingEncoderDirection();

    bool getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Set Encoder counter to zero at next N channel event.
///////////////////////////////////////////////////////////////////////////////
class SetEncoderCounterZeroAtNextNChannel : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    SetEncoderCounterZeroAtNextNChannel();

    virtual ~SetEncoderCounterZeroAtNextNChannel();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Set encoder counter to zero at next switch event.
///////////////////////////////////////////////////////////////////////////////
class SetEncoderCounterZeroAtNextSwitch : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    SetEncoderCounterZeroAtNextSwitch();

    virtual ~SetEncoderCounterZeroAtNextSwitch();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// 1: Set encoder counter zero only once NULL
/// 0: always at an N channel event, respectively switch event.
///////////////////////////////////////////////////////////////////////////////
class SetEncoderCounterZeroOnlyOnce : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    SetEncoderCounterZeroOnlyOnce();

    virtual ~SetEncoderCounterZeroOnlyOnce();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Velocity for sine initialization. [rpm]
///////////////////////////////////////////////////////////////////////////////
class SineInitializationVelocity : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    SineInitializationVelocity();

    virtual ~SineInitializationVelocity();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Stop switch polarity.
/// Bit 0: Left stop switch polarity\n
/// Bit set: Left stop switch input is high active \n
/// Bit clear: Left stop switch input is low active\n\n
/// Bit 1: Right stop switch polarity\n
/// Bit set: Right stop switch input is high active\n
/// Bit clear: Right stop switch input is low active
///////////////////////////////////////////////////////////////////////////////
class StopSwitchPolarity : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    StopSwitchPolarity();

    virtual ~StopSwitchPolarity();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Thermal winding time constant for the used motor. Used for I2t monitoring.
///////////////////////////////////////////////////////////////////////////////
class ThermalWindingTimeConstant : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    ThermalWindingTimeConstant();

    virtual ~ThermalWindingTimeConstant();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::time> upperLimit;

    quantity<si::time> lowerLimit;

    quantity<si::time> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// If the actual speed is below this value the motor halted flag will be set. [rpm]

///////////////////////////////////////////////////////////////////////////////
class MotorHaltedVelocity : public YouBotJointParameterPasswordProtected {
friend class YouBotJoint;
  public:
    MotorHaltedVelocity();

    virtual ~MotorHaltedVelocity();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};

} // namespace youbot
#endif
