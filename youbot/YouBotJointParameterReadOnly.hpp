#ifndef YOUBOT_YOUBOTJOINTPARAMETERREADONLY_H
#define YOUBOT_YOUBOTJOINTPARAMETERREADONLY_H

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
#include "generic-joint/JointParameter.hpp"
#include "youbot/ProtocolDefinitions.hpp"
#include "youbot/YouBotSlaveMsg.hpp"
#include "youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot/YouBotJointStorage.hpp"
namespace youbot {

enum ParameterType {
  MOTOR_CONTOLLER_PARAMETER,
  API_PARAMETER

};
class YouBotJointParameterReadOnly : public JointParameter {
friend class YouBotJoint;
  protected:
    YouBotJointParameterReadOnly();


  public:
    virtual ~YouBotJointParameterReadOnly();


  protected:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const = 0;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) = 0;

    virtual std::string getName() const = 0;

    virtual ParameterType getType() const = 0;

    std::string name;

    ParameterType parameterType;

};
// 0: Enable the standard PID calculation. 
// 1: Disable the PID calculation. The Motor PWM is then directly derived from the target  velocity. 
// 2: Enable an integrating PID algorithm. In this case the result of the PID calculation is integrated to the PWM value. This PID regulation is easier to use than the standard PID regulation. 

class ArePIDcontrollersActive : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ArePIDcontrollersActive();

    virtual ~ArePIDcontrollersActive();

    void getParameter(bool& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
//Actual supply voltage.

class ActualMotorVoltage : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ActualMotorVoltage();

    virtual ~ActualMotorVoltage();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
// Actual PWM duty cycle (0%... 100%).


class ActualPWMDutyCycle : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ActualPWMDutyCycle();

    virtual ~ActualPWMDutyCycle();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
// Bit 0: Overcurrent flag. This flag is set if overcurrent limit is exceeded.
// Bit 1: Undervoltage flag. This flag is set if supply voltage to low for motor operation.
// Bit 2: Overvoltage flag. This flag is set if the motor becomes switched off due to overvoltage.
// Bit 3: Overtemperature flag. This flag is set if overtemperature limit is exceeded.
// Bit 4: Motor halted flag. This flag is set if motor has been switched off.
// Bit 5: Hall error flag. This flag is set upon a hall error.
// Bit 6: Encoder error flag. This flag is set upon an encoder error.
// Bit 7: Winding error flag. [currently not used]
// Bit 8: Cycle time violation. [currently not used]
// Bit 9: Initialization error of sine commutation. This flag is set if initialization is failed.
// Bit 10: Position mode flag. This flag is set when the module is in positioning mode.
// Bit 11: Position end flag. This flag becomes set if the motor has been stopped at the end position.

class ErrorAndStatus : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ErrorAndStatus();

    virtual ~ErrorAndStatus();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    void parseYouBotErrorFlags() const;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
// 0: PWM chopper on high side, HI on low side
// 1: PWM chopper on low side, HI on high
// 2: PWM chopper on low side and high side

class BlockPWMScheme : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    BlockPWMScheme();

    virtual ~BlockPWMScheme();

    void getParameter(int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int value;

    std::string name;

    ParameterType parameterType;

};
// Actual error of PID position regulator

class PositionError : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    PositionError();

    virtual ~PositionError();

    void getParameter(quantity<plane_angle>& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<plane_angle> value;

    std::string name;

    ParameterType parameterType;

};
// Sums of errors of PID position regulator

class PositionErrorSum : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    PositionErrorSum();

    virtual ~PositionErrorSum();

    void getParameter(quantity<plane_angle>& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<plane_angle> value;

    std::string name;

    ParameterType parameterType;

};
// Actual error of PID velocity regulator

class VelocityError : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    VelocityError();

    virtual ~VelocityError();

    void getParameter(quantity<si::angular_velocity>& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
// Sums of Errors of PID velocity regulator
class VelocityErrorSum : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    VelocityErrorSum();

    virtual ~VelocityErrorSum();

    void getParameter(quantity<si::angular_velocity>& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
// 0: Block commutation with hall sensors mode
// 1: Sensorless block commutation (hallFX)
// 2: Sine commutation with hall sensors
// 3: Sine commutation with encoder
// 4: Controlled block commutation
// 5: Controlled sine commutation

class CommutationMode : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    CommutationMode();

    virtual ~CommutationMode();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
// 0: PWM chopper on high side, HI on low side
// 1: PWM chopper on low side, HI on high
// 2: PWM chopper on low side and high side

class PWMSchemeBlockCommutation : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    PWMSchemeBlockCommutation();

    virtual ~PWMSchemeBlockCommutation();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
// Resistance of motor coil. Used for current resistance regulation, position regulation and velocity regulation.

class MotorCoilResistance : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    MotorCoilResistance();

    virtual ~MotorCoilResistance();

    void getParameter(quantity<resistance>& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<resistance> value;

    std::string name;

    ParameterType parameterType;

};
// 0: Initialization in controlled sine commutation
// 1: Initialization in block commutation by using hall sensors

class InitializationMode : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    InitializationMode();

    virtual ~InitializationMode();

    void getParameter(int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int value;

    std::string name;

    ParameterType parameterType;

};
// Encoder Steps per Rotation.

class EncoderResolution : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    EncoderResolution();

    virtual ~EncoderResolution();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
// Encoder direction Set this flag in a way, that turn right increases position counter.

class ReversingEncoderDirection : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ReversingEncoderDirection();

    virtual ~ReversingEncoderDirection();

    bool getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
// Number of motor poles.

class MotorPoles : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    MotorPoles();

    virtual ~MotorPoles();

    void getParameter(bool& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
//1: Hall sensor invert. Sets one of the motors invert with inverted hall scheme, e.g. some Maxon motors

class HallSensorPolarityReversal : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    HallSensorPolarityReversal();

    virtual ~HallSensorPolarityReversal();

    void getParameter(unsigned int& parameter) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};

} // namespace youbot
#endif
