#ifndef YOUBOT_YOUBOTGRIPPERPARAMETER_H
#define YOUBOT_YOUBOTGRIPPERPARAMETER_H

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
#include "youbot_driver/generic-joint/JointParameter.hpp"
#include "youbot_driver/generic-gripper/GripperParameter.hpp"
#include "youbot_driver/youbot/ProtocolDefinitions.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract youBot gripper parameter
///////////////////////////////////////////////////////////////////////////////
class YouBotGripperParameter : public GripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  protected:
    YouBotGripperParameter();


  public:
    virtual ~YouBotGripperParameter();

    virtual void toString(std::string& value) const = 0;


  protected:
    virtual ParameterType getType() const = 0;

    virtual std::string getName() const = 0;

    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const = 0;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) = 0;

    std::string name;


  private:
    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// the firmware version of the gripper
///////////////////////////////////////////////////////////////////////////////
class GripperFirmwareVersion : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    GripperFirmwareVersion();

    virtual ~GripperFirmwareVersion();

    void getParameter(int& controllerType, double& firmwareVersion) const;

    void setParameter(const int controllerType, const double firmwareVersion);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int controllerType;

    double firmwareVersion;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The name for a gripper bar or finger
///////////////////////////////////////////////////////////////////////////////
class GripperBarName : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;


  public:
    GripperBarName();

    virtual ~GripperBarName();

    void getParameter(std::string& parameter) const;

    void setParameter(const std::string parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    std::string value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Calibrate the gripper
///////////////////////////////////////////////////////////////////////////////
class CalibrateGripper : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    CalibrateGripper();

    virtual ~CalibrateGripper();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    ParameterType getType() const {return this->parameterType;};

    std::string getName() const {return this->name;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Represents a bar spacing offset. It could be useful if the gripper can not be totally closed.
///////////////////////////////////////////////////////////////////////////////
class BarSpacingOffset : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    BarSpacingOffset();

    virtual ~BarSpacingOffset();

    void getParameter(quantity<si::length>& parameter) const;

    void setParameter(const quantity<si::length>& parameter);

    void toString(std::string& value) const;


  private:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    ParameterType getType() const {return this->parameterType;};

    std::string getName() const {return this->name;};

    quantity<si::length> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The encoder value when the gripper has reached it's maximum bar spacing position
///////////////////////////////////////////////////////////////////////////////
class MaxEncoderValue : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    MaxEncoderValue();

    virtual ~MaxEncoderValue();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);

    void toString(std::string& value) const;


  private:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    ParameterType getType() const {return this->parameterType;};

    std::string getName() const {return this->name;};

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The maximum bar spacing distance of the gripper
///////////////////////////////////////////////////////////////////////////////
class MaxTravelDistance : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    MaxTravelDistance();

    virtual ~MaxTravelDistance();

    void getParameter(quantity<si::length>& parameter) const;

    void setParameter(const quantity<si::length>& parameter);

    void toString(std::string& value) const;


  private:
    virtual void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    virtual void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    ParameterType getType() const {return this->parameterType;};

    std::string getName() const {return this->name;};

    quantity<si::length> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Actual position of one gripper bar
///////////////////////////////////////////////////////////////////////////////
class ActualPosition : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ActualPosition();

    virtual ~ActualPosition();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Position setpoint for one gripper bar
///////////////////////////////////////////////////////////////////////////////
class PositionSetpoint : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    PositionSetpoint();

    virtual ~PositionSetpoint();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Indicates that the actual position equals the target position. 
///////////////////////////////////////////////////////////////////////////////
class TargetPositionReached : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    TargetPositionReached();

    virtual ~TargetPositionReached();

    void getParameter(bool& parameter) const;

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Actual velocity of one gripper bar
///////////////////////////////////////////////////////////////////////////////
class ActualVelocity : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ActualVelocity();

    virtual ~ActualVelocity();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Velocity setpoint for one gripper bar
///////////////////////////////////////////////////////////////////////////////
class VelocitySetpoint : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    VelocitySetpoint();

    virtual ~VelocitySetpoint();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Readout of the actual load value with used for stall detection (stallGuard2).


///////////////////////////////////////////////////////////////////////////////
class ActualLoadValue : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ActualLoadValue();

    virtual ~ActualLoadValue();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Selects the comparator blank time. This time needs to safely cover the switching event and the duration of the ringing on the sense resistor. For low current drivers, a setting of 1 or 2 is good.

///////////////////////////////////////////////////////////////////////////////
class ChopperBlankTime : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ChopperBlankTime();

    virtual ~ChopperBlankTime();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Hysteresis decrement setting. This setting determines the slope of the hysteresis during on time and during fast decay time.
/// 0   fast decrement
/// 3   very slow decrement

///////////////////////////////////////////////////////////////////////////////
class ChopperHysteresisDecrement : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ChopperHysteresisDecrement();

    virtual ~ChopperHysteresisDecrement();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
///  Hysteresis end setting. Sets the hysteresis end value after a number of decrements. Decrement interval time is controlled by axis parameter 164.
///  -3... -1 negative hysteresis end setting
///  0   zero hysteresis end setting
///  1... 12 positive hysteresis end setting
///////////////////////////////////////////////////////////////////////////////
class ChopperHysteresisEnd : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ChopperHysteresisEnd();

    virtual ~ChopperHysteresisEnd();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
///  Hysteresis start setting. Please remark, that this  value is an offset to the hysteresis end value.
///////////////////////////////////////////////////////////////////////////////
class ChopperHysteresisStart : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ChopperHysteresisStart();

    virtual ~ChopperHysteresisStart();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Selection of the chopper mode:
/// 0   spread cycle
/// 1   classic const. off time

///////////////////////////////////////////////////////////////////////////////
class ChopperMode : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ChopperMode();

    virtual ~ChopperMode();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The off time setting controls the minimum chopper frequency. An off time within the range of 5 s to 20 s will fit. 
/// Off time setting for constant tOFF chopper:
/// NCLK= 12 + 32*tOFF (Minimum is 64 clocks)
//// Setting this parameter to zero completely disables all driver transistors and the motor can free-wheel.

///////////////////////////////////////////////////////////////////////////////
class ChopperOffTime : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ChopperOffTime();

    virtual ~ChopperOffTime();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
///  Every edge of the cycle releases a step/microstep. It does not make sense to activate this parameter for internal use. Double step enable can be used with Step/Dir interface.
/// 0   double step off
/// 1   double step on

///////////////////////////////////////////////////////////////////////////////
class DoubleStepEnable : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;


  public:
    DoubleStepEnable();

    virtual ~DoubleStepEnable();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Bit 0: stallGuardTM status
/// (1: threshold reached)
/// Bit 1: Overtemperature
/// (1: driver is shut down due to overtemperature)
/// Bit 2: Pre-warning overtemperature
/// (1: Threshold is exceeded)
/// Bit 3: Short to ground A
/// (1: Short condition etected, driver currently shut down)
/// Bit 4: Short to ground B
/// (1: Short condition detected, driver currently shut down)
/// Bit 5: Open load A
/// (1: no chopper event has happened during the last period with constant coil polarity)
/// Bit 6: Open load B
/// (1: no chopper event has happened during the last period with constant coil polarity)
/// Bit 7: Stand still
/// (1: No step impulse occurred on the step input during the last 2^20 clock cycles)
///////////////////////////////////////////////////////////////////////////////
class ErrorFlags : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ErrorFlags();

    virtual ~ErrorFlags();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Time after which the power to the motor will be cut when its velocity has reached zero.
/// 0... 65535
/// 0 = never
/// [msec]


///////////////////////////////////////////////////////////////////////////////
class Freewheeling : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    Freewheeling();

    virtual ~Freewheeling();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Acceleration parameter for velocity control and position control
///////////////////////////////////////////////////////////////////////////////
class MaximumAcceleration : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    MaximumAcceleration();

    virtual ~MaximumAcceleration();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The most important motor setting, since too high values might cause motor damage! The maximum value is 255. This value means 100% of the maximum current of the module. The current adjustment is within the range 0... 255 and can be adjusted in 32 steps (0... 255 divided by eight; e.g. step 0 = 0... 7, step 1 = 8... 15 and so on).

///////////////////////////////////////////////////////////////////////////////
class MaximumCurrent : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    MaximumCurrent();

    virtual ~MaximumCurrent();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The limit for acceleration (and deceleration). Changing this parameter requires re-calculation of the acceleration factor (no. 146) and the acceleration divisor (no. 137), which is done automatically.
///////////////////////////////////////////////////////////////////////////////
class MaximumPositioningSpeed : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    MaximumPositioningSpeed();

    virtual ~MaximumPositioningSpeed();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// 0: full step
/// 1: half step
/// 2: 4 microsteps
/// 3: 8 microsteps
/// 4: 16 microsteps
/// 5: 32 microsteps
/// 6: 64 microsteps
/// 7: 128 microsteps
/// 8: 256 microsteps
///////////////////////////////////////////////////////////////////////////////
class MicrostepResolution : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;

  public:
    MicrostepResolution();

    virtual ~MicrostepResolution();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Standstill period before the current is changed down to standby current. The standard value is 200 (value equates 2000msec).
///////////////////////////////////////////////////////////////////////////////
class PowerDownDelay : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    PowerDownDelay();

    virtual ~PowerDownDelay();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The exponent of the scaling factor for the pulse (step) generator   should be de/incremented carefully (in steps of one).
///////////////////////////////////////////////////////////////////////////////
class PulseDivisor : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;


  public:
    PulseDivisor();

    virtual ~PulseDivisor();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The exponent of the scaling factor for the ramp generator- should be de/incremented carefully (in steps of one).
///////////////////////////////////////////////////////////////////////////////
class RampDivisor : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;

  public:
    RampDivisor();

    virtual ~RampDivisor();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Automatically set when using ROR, ROL, MST and MVP.
/// 0: position mode. Steps are generated, when the parameters actual position and target position differ. Trapezoidal speed ramps are provided.
/// 2: velocity mode. The motor will run continuously and the speed will be changed with constant (maximum) acceleration, if the parameter target speed is changed. For special purposes, the soft mode (value 1) with exponential decrease of speed can be selected.
///////////////////////////////////////////////////////////////////////////////
class RampMode : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;

  public:
    RampMode();

    virtual ~RampMode();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// 0: 3.2 s
/// 1: 1.6 s
/// 2: 1.2 s
/// 3: 0.8 s
/// Use default value!


///////////////////////////////////////////////////////////////////////////////
class ShortDetectionTimer : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ShortDetectionTimer();

    virtual ~ShortDetectionTimer();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// 0: Short to GND protection is on
/// 1: Short to GND protection is disabled
/// Use default value!

///////////////////////////////////////////////////////////////////////////////
class ShortProtectionDisable : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ShortProtectionDisable();

    virtual ~ShortProtectionDisable();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Determines the slope of the motor driver outputs. Set to 2 or 3 for this module or rather use the default value.
/// 0: lowest slope
/// 3: fastest slope

///////////////////////////////////////////////////////////////////////////////
class SlopeControlHighSide : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SlopeControlHighSide();

    virtual ~SlopeControlHighSide();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Determines the slope of the motor driver outputs. Set identical to slope control high side.

///////////////////////////////////////////////////////////////////////////////
class SlopeControlLowSide : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SlopeControlLowSide();

    virtual ~SlopeControlLowSide();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// This status value provides the actual motor current setting as controlled by coolStepTM. The value goes up to the CS value and down to the portion of CS as specified by SEIMIN.
/// actual motor current scaling factor:
/// 0 ... 31: 1/32, 2/32, ... 32/32


///////////////////////////////////////////////////////////////////////////////
class SmartEnergyActualCurrent : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyActualCurrent();

    virtual ~SmartEnergyActualCurrent();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sets the number of stallGuard2 readings current down above the upper threshold necessary for each step current decrement of the motor current.
/// Number of stallGuard2 measurements per decrement:
/// Scaling: 0... 3: 32, 8, 2, 1
/// 0: slow decrement
/// 3: fast decrement
///////////////////////////////////////////////////////////////////////////////
class SmartEnergyCurrentDownStep : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyCurrentDownStep();

    virtual ~SmartEnergyCurrentDownStep();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sets the lower motor current limit for current minimum coolStep operation by scaling the CS (Current Scale, see axis parameter 6) value.
/// minimum motor current:
/// 0   1/2 of CS
/// 1   1/4 of CS
///////////////////////////////////////////////////////////////////////////////
class SmartEnergyCurrentMinimum : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyCurrentMinimum();

    virtual ~SmartEnergyCurrentMinimum();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sets the current increment step. The current becomes incremented for each measured stallGuard2 value below the lower threshold (see smartEnergy hysteresis start). current increment step size:
/// Scaling: 0... 3: 1, 2, 4, 8
/// 0: slow increment
/// 3: fast increment / fast reaction to rising load

///////////////////////////////////////////////////////////////////////////////
class SmartEnergyCurrentUpStep : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyCurrentUpStep();

    virtual ~SmartEnergyCurrentUpStep();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sets the distance between the lower and the upper threshold for stallGuard2TM reading. Above the upper threshold the motor current becomes decreased.
/// Hysteresis: (smartEnergy hysteresis value + 1) * 32
/// Upper stallGuard2 threshold: (smartEnergy hysteresis start + smartEnergy hysteresis + 1) * 32
///////////////////////////////////////////////////////////////////////////////
class SmartEnergyHysteresis : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyHysteresis();

    virtual ~SmartEnergyHysteresis();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The lower threshold for the stallGuard2 value (see smart Energy current up step).

///////////////////////////////////////////////////////////////////////////////
class SmartEnergyHysteresisStart : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyHysteresisStart();

    virtual ~SmartEnergyHysteresisStart();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sets the motor current which is used below the threshold speed.

///////////////////////////////////////////////////////////////////////////////
class SmartEnergySlowRunCurrent : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergySlowRunCurrent();

    virtual ~SmartEnergySlowRunCurrent();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Above this speed coolStep becomes enabled.

///////////////////////////////////////////////////////////////////////////////
class SmartEnergyThresholdSpeed : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    SmartEnergyThresholdSpeed();

    virtual ~SmartEnergyThresholdSpeed();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Enables the stallGuard2 filter for more precision of the measurement. If set, reduces the measurement frequency to one measurement per four fullsteps. In most cases it is expedient to set the filtered mode before using coolStep. Use the standard mode for step loss detection.
/// 0   standard mode
/// 1   filtered mode
///////////////////////////////////////////////////////////////////////////////
class StallGuard2FilterEnable : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    StallGuard2FilterEnable();

    virtual ~StallGuard2FilterEnable();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// This signed value controls stallGuard2 threshold level for stall output and sets the optimum measurement range for readout. A lower value gives a higher sensitivity. Zero is the starting value. A higher value makes stallGuard2 less sensitive and requires more torque to indicate a stall. 
/// 0 Indifferent value
/// 1... 63 less sensitivity
/// -1... -64 higher sensitivity
///////////////////////////////////////////////////////////////////////////////
class StallGuard2Threshold : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    StallGuard2Threshold();

    virtual ~StallGuard2Threshold();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The current limit two seconds after the motor has stopped.

///////////////////////////////////////////////////////////////////////////////
class StandbyCurrent : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;

  public:
    StandbyCurrent();

    virtual ~StandbyCurrent();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Step interpolation is supported with a 16 microstep setting only. In this setting, each step impulse at the input causes the execution of 16 times 1/256 microsteps. This way, a smooth motor movement like in 256 microstep resolution is achieved.
/// 0   step interpolation off
/// 1   step interpolation on

///////////////////////////////////////////////////////////////////////////////
class StepInterpolationEnable : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;


  public:
    StepInterpolationEnable();

    virtual ~StepInterpolationEnable();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Motor stop in case of stall.
///////////////////////////////////////////////////////////////////////////////
class StopOnStall : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    StopOnStall();

    virtual ~StopOnStall();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    bool value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// sense resistor voltage based current scaling
/// 0: Full scale sense resistor voltage is 1/18 VDD
/// 1: Full scale sense resistor voltage is 1/36 VDD
/// (refers to a current setting of 31 and DAC value 255)
/// Use default value. Do not change!

///////////////////////////////////////////////////////////////////////////////
class Vsense : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    Vsense();

    virtual ~Vsense();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int& parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    unsigned int upperLimit;

    unsigned int lowerLimit;

    unsigned int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The current acceleration (read only).
///////////////////////////////////////////////////////////////////////////////
class ActualAcceleration : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    ActualAcceleration();

    virtual ~ActualAcceleration();

    void getParameter(int& parameter) const;

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    int upperLimit;

    int lowerLimit;

    int value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Should  always  be  set  1  to  ensure  exact reaching  of  the  target  position.  Do  not change!
///////////////////////////////////////////////////////////////////////////////
class MinimumSpeed : public YouBotGripperParameter {
friend class YouBotGripper;
friend class YouBotGripperBar;
  public:
    MinimumSpeed();

    virtual ~MinimumSpeed();

    void getParameter(int& parameter) const;

    void setParameter(const int parameter);

    void toString(std::string& value) const;


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message);

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
