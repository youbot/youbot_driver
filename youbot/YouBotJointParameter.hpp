#ifndef YOUBOT_YOUBOTJOINTPARAMETER_H
#define YOUBOT_YOUBOTJOINTPARAMETER_H

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
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "rude/config.h"
#include "generic/Exceptions.hpp"
#include "generic-joint/JointParameter.hpp"
#include "youbot/YouBotJointParameterReadOnly.hpp"
#include "youbot/ProtocolDefinitions.hpp"
#include "youbot/YouBotSlaveMsg.hpp"
#include "youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot/YouBotJointStorage.hpp"
namespace youbot {

enum CalibrationDirection {
  POSITIV,
  NEGATIV

};
class YouBotJointParameter : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  protected:
    YouBotJointParameter();


  public:
    virtual ~YouBotJointParameter();


  protected:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const = 0;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) = 0;

    virtual std::string getName() const = 0;

    virtual ParameterType getType() const = 0;

    std::string name;

    ParameterType parameterType;

};
class JointName : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    JointName();

    virtual ~JointName();

    void getParameter(std::string& parameter) const;

    void setParameter(const std::string parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    std::string value;

    std::string name;

    ParameterType parameterType;

};
class GearRatio : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    GearRatio();

    virtual ~GearRatio();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double value;

    std::string name;

    ParameterType parameterType;

};
class EncoderTicksPerRound : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    EncoderTicksPerRound();

    virtual ~EncoderTicksPerRound();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
class InverseMovementDirection : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    InverseMovementDirection();

    virtual ~InverseMovementDirection();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
class CalibrateJoint : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    CalibrateJoint();

    virtual ~CalibrateJoint();

    void getParameter(bool& doCalibration, CalibrationDirection& calibrationDirection, quantity<si::current>& maxCurrent) const;

    void setParameter(const bool doCalibration, CalibrationDirection calibrationDirection, const quantity<si::current>& maxCurrent);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool doCalibration;

    std::string name;

    ParameterType parameterType;

    CalibrationDirection calibrationDirection;

    quantity<si::current> maxCurrent;

};
class JointLimits : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    JointLimits();

    virtual ~JointLimits();

    void getParameter(int& lowerLimit, int& upperLimit) const;

    void setParameter(const int lowerLimit, const int upperLimit);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int lowerLimit;

    int upperLimit;

    std::string name;

    ParameterType parameterType;

};
class StopJoint : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    StopJoint();

    virtual ~StopJoint();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
class NoMoreAction : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    NoMoreAction();

    virtual ~NoMoreAction();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
//The maximum velocity used for move to position command when executing a ramp to a position. In sensorless commutation mode the velocity threshold for hallFXTM.

class MaximumPositioningVelocity : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MaximumPositioningVelocity();

    virtual ~MaximumPositioningVelocity();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_velocity> upperLimit;

    quantity<angular_velocity> lowerLimit;

    quantity<angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
//Set PWM limit (0%... 100%).

class PWMLimit : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PWMLimit();

    virtual ~PWMLimit();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);


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
//Switching threshold for speed control between the first and second set of parameters
class MaximumMotorCurrent : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MaximumMotorCurrent();

    virtual ~MaximumMotorCurrent();

    void getParameter(quantity<current>& parameter) const;

    void setParameter(const quantity<current>& parameter);


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
//Maximum velocity at which end position can be set. Prevents issuing of end position when the target is passed at high velocity

class MaximumVelocityToSetPosition : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MaximumVelocityToSetPosition();

    virtual ~MaximumVelocityToSetPosition();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_velocity> upperLimit;

    quantity<angular_velocity> lowerLimit;

    quantity<angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
//Adjusts the limit to switch between first velocity PID parameter set and second velocity PID parameter set.

class SpeedControlSwitchingThreshold : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    SpeedControlSwitchingThreshold();

    virtual ~SpeedControlSwitchingThreshold();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_velocity> upperLimit;

    quantity<angular_velocity> lowerLimit;

    quantity<angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
class ClearTargetDistance : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ClearTargetDistance();

    virtual ~ClearTargetDistance();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class PositionTargetReachedDistance : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PositionTargetReachedDistance();

    virtual ~PositionTargetReachedDistance();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Switching threshold for speed control between the first and second set of parameters
class MotorAcceleration : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MotorAcceleration();

    virtual ~MotorAcceleration();

    void getParameter(quantity<angular_acceleration>& parameter) const;

    void setParameter(const quantity<angular_acceleration>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_acceleration> upperLimit;

    quantity<angular_acceleration> lowerLimit;

    quantity<angular_acceleration> value;

    std::string name;

    ParameterType parameterType;

};
//Switching threshold for position control between the first and second set of parameters
class PositionControlSwitchingThreshold : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PositionControlSwitchingThreshold();

    virtual ~PositionControlSwitchingThreshold();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_velocity> upperLimit;

    quantity<angular_velocity> lowerLimit;

    quantity<angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
class PParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterFirstParametersPositionControl();

    virtual ~PParameterFirstParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterFirstParametersPositionControl();

    virtual ~IParameterFirstParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class DParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterFirstParametersPositionControl();

    virtual ~DParameterFirstParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//PID calculation delay: Set operational frequency PID

class PIDControlTime : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PIDControlTime();

    virtual ~PIDControlTime();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);


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
//PID calculation delay: Set operational frequency PID

class CurrentControlLoopDelay : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    CurrentControlLoopDelay();

    virtual ~CurrentControlLoopDelay();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);


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
//Adjust in standstill to lowest possible value at which the motor keeps its position. A too high value causes overshooting at positioning mode. (first position parameter set)

class IClippingParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterFirstParametersPositionControl();

    virtual ~IClippingParameterFirstParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Compensates dead time of PWM and motor friction.

class PWMHysteresis : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PWMHysteresis();

    virtual ~PWMHysteresis();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Compensates dead time of PWM and motor friction.

class ClearISumIfPWMReachesMaximum : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ClearISumIfPWMReachesMaximum();

    virtual ~ClearISumIfPWMReachesMaximum();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
//Compensates dead time of PWM and motor friction.

class ClearISumIfOvershootsTarget : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ClearISumIfOvershootsTarget();

    virtual ~ClearISumIfOvershootsTarget();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
class PParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterFirstParametersSpeedControl();

    virtual ~PParameterFirstParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterFirstParametersSpeedControl();

    virtual ~IParameterFirstParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class DParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterFirstParametersSpeedControl();

    virtual ~DParameterFirstParametersSpeedControl();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);


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
class IClippingParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterFirstParametersSpeedControl();

    virtual ~IClippingParameterFirstParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Switches the ramp generator for speed and position control on and off
class RampGeneratorSpeedAndPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    RampGeneratorSpeedAndPositionControl();

    virtual ~RampGeneratorSpeedAndPositionControl();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
//Enabled the re-initialization of the sinusoidal commutation
class ReinitializationSinusoidalCommutation : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ReinitializationSinusoidalCommutation();

    virtual ~ReinitializationSinusoidalCommutation();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
// 1: Set Encoder counter to zero at next N channel event.

class SetEncoderCounterZeroAtNextNChannel : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    SetEncoderCounterZeroAtNextNChannel();

    virtual ~SetEncoderCounterZeroAtNextNChannel();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
// 1: Set encoder counter to zero at next switch event.

class SetEncoderCounterZeroAtNextSwitch : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    SetEncoderCounterZeroAtNextSwitch();

    virtual ~SetEncoderCounterZeroAtNextSwitch();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
// 1: Set encoder counter zero only once NULL
// 0: always at an N channel event, respectively switch event.
class SetEncoderCounterZeroOnlyOnce : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    SetEncoderCounterZeroOnlyOnce();

    virtual ~SetEncoderCounterZeroOnlyOnce();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
class EncoderStopSwitch : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    EncoderStopSwitch();

    virtual ~EncoderStopSwitch();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);


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
class ActualCommutationOffset : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ActualCommutationOffset();

    virtual ~ActualCommutationOffset();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class StopSwitchPolarity : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    StopSwitchPolarity();

    virtual ~StopSwitchPolarity();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);


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
class PParameterFirstParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterFirstParametersCurrentControl();

    virtual ~PParameterFirstParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IParameterFirstParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterFirstParametersCurrentControl();

    virtual ~IParameterFirstParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class DParameterFirstParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterFirstParametersCurrentControl();

    virtual ~DParameterFirstParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IClippingParameterFirstParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterFirstParametersCurrentControl();

    virtual ~IClippingParameterFirstParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class PParameterSecondParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterSecondParametersCurrentControl();

    virtual ~PParameterSecondParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IParameterSecondParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterSecondParametersCurrentControl();

    virtual ~IParameterSecondParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class DParameterSecondParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterSecondParametersCurrentControl();

    virtual ~DParameterSecondParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IClippingParameterSecondParametersCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterSecondParametersCurrentControl();

    virtual ~IClippingParameterSecondParametersCurrentControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Switching threshold for position control between the first and second set of parameters
class CurrentControlSwitchingThreshold : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    CurrentControlSwitchingThreshold();

    virtual ~CurrentControlSwitchingThreshold();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_velocity> upperLimit;

    quantity<angular_velocity> lowerLimit;

    quantity<angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
class PParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterSecondParametersPositionControl();

    virtual ~PParameterSecondParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterSecondParametersPositionControl();

    virtual ~IParameterSecondParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class DParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterSecondParametersPositionControl();

    virtual ~DParameterSecondParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IClippingParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterSecondParametersPositionControl();

    virtual ~IClippingParameterSecondParametersPositionControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class PParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterSecondParametersSpeedControl();

    virtual ~PParameterSecondParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterSecondParametersSpeedControl();

    virtual ~IParameterSecondParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class DParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterSecondParametersSpeedControl();

    virtual ~DParameterSecondParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class IClippingParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterSecondParametersSpeedControl();

    virtual ~IClippingParameterSecondParametersSpeedControl();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class MassInertiaConstant : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MassInertiaConstant();

    virtual ~MassInertiaConstant();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
class BEMFConstant : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    BEMFConstant();

    virtual ~BEMFConstant();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Maximum speed for initialisation in block commutation
class BlockCommutationMaximumSpeed : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    BlockCommutationMaximumSpeed();

    virtual ~BlockCommutationMaximumSpeed();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<angular_velocity> upperLimit;

    quantity<angular_velocity> lowerLimit;

    quantity<angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
//Commutation (CW) to compensate for the Hall sensor deviations. The aim is that the motor rotates in either direction with equal speed.
class CommutationCompensationClockwise : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    CommutationCompensationClockwise();

    virtual ~CommutationCompensationClockwise();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Commutation (CCW) to compensate for the Hall sensor deviations. The aim is that the motor rotates in either direction with equal speed.
class CommutationCompensationCounterClockwise : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    CommutationCompensationCounterClockwise();

    virtual ~CommutationCompensationCounterClockwise();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//PID calculation delay: Set operational frequency PID

class InitSineDelay : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    InitSineDelay();

    virtual ~InitSineDelay();

    void getParameter(quantity<si::time>& parameter) const;

    void setParameter(const quantity<si::time>& parameter);


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
// 1: Set encoder counter zero only once NULL
// 0: always at an N channel event, respectively switch event.
class ActivateOvervoltageProtection : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ActivateOvervoltageProtection();

    virtual ~ActivateOvervoltageProtection();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
//Commutation (CCW) to compensate for the Hall sensor deviations. The aim is that the motor rotates in either direction with equal speed.
class MaximumPWMChangePerPIDInterval : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MaximumPWMChangePerPIDInterval();

    virtual ~MaximumPWMChangePerPIDInterval();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
//Commutation (CCW) to compensate for the Hall sensor deviations. The aim is that the motor rotates in either direction with equal speed.
class SineCompensationFactor : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    SineCompensationFactor();

    virtual ~SineCompensationFactor();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);


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
// 1: Set encoder counter zero only once NULL
// 0: always at an N channel event, respectively switch event.
class EncoderNullPolarity : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    EncoderNullPolarity();

    virtual ~EncoderNullPolarity();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};

} // namespace youbot
#endif
