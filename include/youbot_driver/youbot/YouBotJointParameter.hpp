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
#include <boost/limits.hpp>
#include "youbot_driver/youbot/YouBotJointParameterReadOnly.hpp"
namespace youbot {

enum CalibrationDirection {
  POSITIV,
  NEGATIV

};
///////////////////////////////////////////////////////////////////////////////
/// abstract youBot API joint parameter
///////////////////////////////////////////////////////////////////////////////
class YouBotApiJointParameter : public JointParameter {
friend class YouBotJoint;
  protected:
    YouBotApiJointParameter();


  public:
    virtual ~YouBotApiJointParameter();

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
/// abstract youBot joint parameter
///////////////////////////////////////////////////////////////////////////////
class YouBotJointParameter : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  protected:
    YouBotJointParameter();


  public:
    virtual ~YouBotJointParameter();

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
/// the name of the joint
///////////////////////////////////////////////////////////////////////////////
class JointName : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    JointName();

    virtual ~JointName();

    void getParameter(std::string& parameter) const;

    void setParameter(const std::string parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    std::string value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Initialize Joint
///////////////////////////////////////////////////////////////////////////////
class InitializeJoint : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    InitializeJoint();

    virtual ~InitializeJoint();

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
/// calibrates the joint
///////////////////////////////////////////////////////////////////////////////
class CalibrateJoint : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    CalibrateJoint();

    virtual ~CalibrateJoint();

    void getParameter(bool& doCalibration, CalibrationDirection& calibrationDirection, quantity<si::current>& maxCurrent) const;

    void setParameter(const bool doCalibration, CalibrationDirection calibrationDirection, const quantity<si::current>& maxCurrent);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// the firmware version of the joint
///////////////////////////////////////////////////////////////////////////////
class FirmwareVersion : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    FirmwareVersion();

    virtual ~FirmwareVersion();

    void getParameter(int& controllerType, std::string& firmwareVersion) const;

    void setParameter(const int controllerType, const std::string firmwareVersion);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int controllerType;

    std::string firmwareVersion;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// the gear ratio which is needed for the calculations in the youBot driver
///////////////////////////////////////////////////////////////////////////////
class GearRatio : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    GearRatio();

    virtual ~GearRatio();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// the resolution of the encoders, it is needed for the calculations of the youBot Driver
///////////////////////////////////////////////////////////////////////////////
class EncoderTicksPerRound : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    EncoderTicksPerRound();

    virtual ~EncoderTicksPerRound();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// inverse the joint movement direction
///////////////////////////////////////////////////////////////////////////////
class InverseMovementDirection : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    InverseMovementDirection();

    virtual ~InverseMovementDirection();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// joint position limits in encoder ticks
///////////////////////////////////////////////////////////////////////////////
class JointLimits : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    JointLimits();

    virtual ~JointLimits();

    void getParameter(int& lowerLimit, int& upperLimit, bool& areLimitsActive) const;

    void setParameter(const int lowerLimit, const int upperLimit, const bool activateLimits);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int lowerLimit;

    int upperLimit;

    std::string name;

    ParameterType parameterType;

    bool areLimitsActive;

};
///////////////////////////////////////////////////////////////////////////////
/// joint position limits in radian
///////////////////////////////////////////////////////////////////////////////
class JointLimitsRadian : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    JointLimitsRadian();

    virtual ~JointLimitsRadian();

    void getParameter(quantity<plane_angle>& lowerLimit, quantity<plane_angle>& upperLimit, bool& areLimitsActive) const;

    void setParameter(const quantity<plane_angle>& lowerLimit, const quantity<plane_angle>& upperLimit, const bool activateLimits);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<plane_angle> lowerLimit;

    quantity<plane_angle> upperLimit;

    std::string name;

    ParameterType parameterType;

    bool areLimitsActive;

};
///////////////////////////////////////////////////////////////////////////////
/// the resolution of the encoders, it is needed for the calculations of the youBot Driver
///////////////////////////////////////////////////////////////////////////////
class TorqueConstant : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    TorqueConstant();

    virtual ~TorqueConstant();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {};

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {};

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The maximum velocity used for move to position command when executing a ramp to a position. In sensorless commutation mode the velocity threshold for hallFX. In sensorless commutation mode used as velocity threshold for hallFXTM. Set this value to a realistic velocity which the motor can reach!
///////////////////////////////////////////////////////////////////////////////
class MaximumPositioningVelocity : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MaximumPositioningVelocity();

    virtual ~MaximumPositioningVelocity();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// Acceleration parameter for velocity control and position control
///////////////////////////////////////////////////////////////////////////////
class MotorAcceleration : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MotorAcceleration();

    virtual ~MotorAcceleration();

    void getParameter(quantity<angular_acceleration>& parameter) const;

    void setParameter(const quantity<angular_acceleration>& parameter);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// Switches the ramp generator for speed and position control on and off 
///////////////////////////////////////////////////////////////////////////////
class RampGeneratorSpeedAndPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    RampGeneratorSpeedAndPositionControl();

    virtual ~RampGeneratorSpeedAndPositionControl();

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
/// Switching threshold for position control between the first and second set of parameters. If the velocity threshold is set to zero, the parameter set 2 is used all the time.
///////////////////////////////////////////////////////////////////////////////
class PositionControlSwitchingThreshold : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PositionControlSwitchingThreshold();

    virtual ~PositionControlSwitchingThreshold();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// Adjusts the limit to switch between first velocity PID parameter set and second velocity PID parameter set. If the velocity threshold is set to zero, the parameter set 2 is used all the time.

///////////////////////////////////////////////////////////////////////////////
class SpeedControlSwitchingThreshold : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    SpeedControlSwitchingThreshold();

    virtual ~SpeedControlSwitchingThreshold();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// Velocity to switch from controlled to hallFX mode. Set this value to a realistic velocity which the motor can reach in controlled mode!
///////////////////////////////////////////////////////////////////////////////
class VelocityThresholdForHallFX : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    VelocityThresholdForHallFX();

    virtual ~VelocityThresholdForHallFX();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// P-Parameter of PID position regulator (first position parameter set)
///////////////////////////////////////////////////////////////////////////////
class PParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterFirstParametersPositionControl();

    virtual ~PParameterFirstParametersPositionControl();

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
/// I-Parameter of PID position regulator (first position parameter set)
///////////////////////////////////////////////////////////////////////////////
class IParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterFirstParametersPositionControl();

    virtual ~IParameterFirstParametersPositionControl();

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
/// D-Parameter of PID position regulator (first position parameter set)
///////////////////////////////////////////////////////////////////////////////
class DParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterFirstParametersPositionControl();

    virtual ~DParameterFirstParametersPositionControl();

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
/// Adjust in standstill to lowest possible value at which the motor keeps its position. A too high value causes overshooting at positioning mode. (first position parameter set)
///////////////////////////////////////////////////////////////////////////////
class IClippingParameterFirstParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterFirstParametersPositionControl();

    virtual ~IClippingParameterFirstParametersPositionControl();

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
/// P-Parameter of PID velocity regulator. This PID parameter set is used at lower velocity. (first velocity parameter set) 
///////////////////////////////////////////////////////////////////////////////
class PParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterFirstParametersSpeedControl();

    virtual ~PParameterFirstParametersSpeedControl();

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
/// I-Parameter of PID velocity regulator. This PID parameter set is used at lower velocity. (first velocity parameter set) 
///////////////////////////////////////////////////////////////////////////////
class IParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterFirstParametersSpeedControl();

    virtual ~IParameterFirstParametersSpeedControl();

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
/// D-Parameter of PID velocity regulator. This PID parameter set is used at lower velocity. (first velocity parameter set) 
///////////////////////////////////////////////////////////////////////////////
class DParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterFirstParametersSpeedControl();

    virtual ~DParameterFirstParametersSpeedControl();

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
/// This PID parameter set is used at lower velocity. (first velocity parameter set)
///////////////////////////////////////////////////////////////////////////////
class IClippingParameterFirstParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterFirstParametersSpeedControl();

    virtual ~IClippingParameterFirstParametersSpeedControl();

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
/// P-Parameter of PID position regulator (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class PParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterSecondParametersPositionControl();

    virtual ~PParameterSecondParametersPositionControl();

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
/// I-Parameter of PID position regulator (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class IParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterSecondParametersPositionControl();

    virtual ~IParameterSecondParametersPositionControl();

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
/// D-Parameter of PID position regulator (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class DParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterSecondParametersPositionControl();

    virtual ~DParameterSecondParametersPositionControl();

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
/// Adjust in standstill to lowest possible value at which the motor keeps its position. A too high value causes overshooting at positioning mode. (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class IClippingParameterSecondParametersPositionControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterSecondParametersPositionControl();

    virtual ~IClippingParameterSecondParametersPositionControl();

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
/// P-Parameter of PID velocity regulator (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class PParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterSecondParametersSpeedControl();

    virtual ~PParameterSecondParametersSpeedControl();

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
/// I-Parameter of PID velocity regulator (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class IParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterSecondParametersSpeedControl();

    virtual ~IParameterSecondParametersSpeedControl();

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
/// D-Parameter of PID velocity regulator (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class DParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterSecondParametersSpeedControl();

    virtual ~DParameterSecondParametersSpeedControl();

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
/// I-Clipping Parameter of PID current regulator. This PID parameter set is used at lower velocity. (second position parameter set)
///////////////////////////////////////////////////////////////////////////////
class IClippingParameterSecondParametersSpeedControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterSecondParametersSpeedControl();

    virtual ~IClippingParameterSecondParametersSpeedControl();

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
/// P-Parameter of PID current regulator.
///////////////////////////////////////////////////////////////////////////////
class PParameterCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PParameterCurrentControl();

    virtual ~PParameterCurrentControl();

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
/// I-Parameter of PID current regulator.
///////////////////////////////////////////////////////////////////////////////
class IParameterCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IParameterCurrentControl();

    virtual ~IParameterCurrentControl();

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
/// D-Parameter of PID current regulator.
///////////////////////////////////////////////////////////////////////////////
class DParameterCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    DParameterCurrentControl();

    virtual ~DParameterCurrentControl();

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
/// I-Clipping Parameter of PID current regulator. 
///////////////////////////////////////////////////////////////////////////////
class IClippingParameterCurrentControl : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterCurrentControl();

    virtual ~IClippingParameterCurrentControl();

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
/// Maximum velocity at which end position can be set. Prevents issuing of end position when the target is passed at high velocity
///////////////////////////////////////////////////////////////////////////////
class MaximumVelocityToSetPosition : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    MaximumVelocityToSetPosition();

    virtual ~MaximumVelocityToSetPosition();

    void getParameter(quantity<angular_velocity>& parameter) const;

    void setParameter(const quantity<angular_velocity>& parameter);

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// Maximum distance at which the position end flag is set.
///////////////////////////////////////////////////////////////////////////////
class PositionTargetReachedDistance : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    PositionTargetReachedDistance();

    virtual ~PositionTargetReachedDistance();

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
/// Clear the flag that indicates that the I2t sum has exceeded the I2t limit.
///////////////////////////////////////////////////////////////////////////////
class ClearI2tExceededFlag : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ClearI2tExceededFlag();

    virtual ~ClearI2tExceededFlag();

    void getParameter() const;

    void setParameter();

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
/// Clear the flag that indicates a communication timeout between the EtherCAT master and the controller.

///////////////////////////////////////////////////////////////////////////////
class ClearMotorControllerTimeoutFlag : public YouBotJointParameter {
friend class YouBotJoint;
  public:
    ClearMotorControllerTimeoutFlag();

    virtual ~ClearMotorControllerTimeoutFlag();

    bool getParameter() const;

    void setParameter();

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
/// P-Parameter of PID trajectory regulator
///////////////////////////////////////////////////////////////////////////////
class PParameterTrajectoryControl : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    PParameterTrajectoryControl();

    virtual ~PParameterTrajectoryControl();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);

    void toString(std::string& value);


  private:
    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double upperLimit;

    double lowerLimit;

    double value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// I-Parameter of PID trajectory regulator 
///////////////////////////////////////////////////////////////////////////////
class IParameterTrajectoryControl : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    IParameterTrajectoryControl();

    virtual ~IParameterTrajectoryControl();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);

    void toString(std::string& value);


  private:
    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double upperLimit;

    double lowerLimit;

    double value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// D-Parameter of PID trajectory regulator
///////////////////////////////////////////////////////////////////////////////
class DParameterTrajectoryControl : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    DParameterTrajectoryControl();

    virtual ~DParameterTrajectoryControl();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);

    void toString(std::string& value);


  private:
    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double upperLimit;

    double lowerLimit;

    double value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// gives a limit for the I sum part of the trajectory regulator
///////////////////////////////////////////////////////////////////////////////
class IClippingParameterTrajectoryControl : public YouBotApiJointParameter {
friend class YouBotJoint;
  public:
    IClippingParameterTrajectoryControl();

    virtual ~IClippingParameterTrajectoryControl();

    void getParameter(double& parameter) const;

    void setParameter(const double parameter);

    void toString(std::string& value);


  private:
    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    double upperLimit;

    double lowerLimit;

    double value;

    std::string name;

    ParameterType parameterType;

};

} // namespace youbot
#endif
