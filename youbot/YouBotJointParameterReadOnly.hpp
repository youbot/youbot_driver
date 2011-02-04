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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
// Actual PWM duty cycle (0%... 100%).


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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
//Switches the ramp generator for speed and position control on and off
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
