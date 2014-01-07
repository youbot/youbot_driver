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
#include "youbot_driver/generic/Logger.hpp"
#include "youbot_driver/generic/Units.hpp"
#include "youbot_driver/generic/Time.hpp"
#include "youbot_driver/generic/Exceptions.hpp"
#include "youbot_driver/generic-joint/JointParameter.hpp"
#include "youbot_driver/youbot/ProtocolDefinitions.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/youbot/YouBotSlaveMailboxMsg.hpp"
#include "youbot_driver/youbot/YouBotJointStorage.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract youBot joint parameter which can be read only
///////////////////////////////////////////////////////////////////////////////
class YouBotJointParameterReadOnly : public JointParameter {
friend class YouBotJoint;
  protected:
    YouBotJointParameterReadOnly();


  public:
    virtual ~YouBotJointParameterReadOnly();

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
/// Actual supply voltage.
///////////////////////////////////////////////////////////////////////////////
class ActualMotorVoltage : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ActualMotorVoltage();

    virtual ~ActualMotorVoltage();

    void getParameter(quantity<electric_potential>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    /// testt
    ParameterType getType() const {return this->parameterType;};

    quantity<electric_potential> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Error and Status flags of the joint. 
/// Bit 0: Overcurrent flag. This flag is set if overcurrent limit is exceeded. \n
/// Bit 1: Undervoltage flag. This flag is set if supply voltage to low for motor operation. \n
/// Bit 2: Overvoltage flag. This flag is set if the motor becomes switched off due to overvoltage. \n
/// Bit 3: Overtemperature flag. This flag is set if overtemperature limit is exceeded. \n
/// Bit 4: Motor halted flag. This flag is set if motor has been switched off. \n
/// Bit 5: Hall error flag. This flag is set upon a hall error. \n
/// Bit 6: Encoder error flag. This flag is set upon an encoder error. \n
/// Bit 7: Winding error flag. [currently not used] \n
/// Bit 8: Cycle time violation. [currently not used] \n
/// Bit 9: Initialization error of sine commutation. This flag is set if initialization is failed. \n
/// Bit 10: Position mode flag. This flag is set when the module is in positioning mode. \n
/// Bit 11: Position end flag. This flag becomes set if the motor has been stopped at the end position. \n
///////////////////////////////////////////////////////////////////////////////
class ErrorAndStatus : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ErrorAndStatus();

    virtual ~ErrorAndStatus();

    void getParameter(unsigned int& parameter) const;

    void toString(std::string& value);


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
///////////////////////////////////////////////////////////////////////////////
/// Actual error of PID position regulator
///////////////////////////////////////////////////////////////////////////////
class PositionError : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    PositionError();

    virtual ~PositionError();

    void getParameter(quantity<plane_angle>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<plane_angle> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sums of errors of PID position regulator
///////////////////////////////////////////////////////////////////////////////
class PositionErrorSum : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    PositionErrorSum();

    virtual ~PositionErrorSum();

    void getParameter(quantity<plane_angle>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<plane_angle> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Actual error of PID velocity regulator
///////////////////////////////////////////////////////////////////////////////
class VelocityError : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    VelocityError();

    virtual ~VelocityError();

    void getParameter(quantity<si::angular_velocity>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sums of Errors of PID velocity regulator
///////////////////////////////////////////////////////////////////////////////
class VelocityErrorSum : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    VelocityErrorSum();

    virtual ~VelocityErrorSum();

    void getParameter(quantity<si::angular_velocity>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Actual error of current PID regulator
///////////////////////////////////////////////////////////////////////////////
class CurrentError : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    CurrentError();

    virtual ~CurrentError();

    void getParameter(quantity<si::current>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::current> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Sum of errors of current PID regulator
///////////////////////////////////////////////////////////////////////////////
class CurrentErrorSum : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    CurrentErrorSum();

    virtual ~CurrentErrorSum();

    void getParameter(quantity<si::current>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::current> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// The actual speed of the velocity ramp used for positioning and velocity mode. 
///////////////////////////////////////////////////////////////////////////////
class RampGeneratorSpeed : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    RampGeneratorSpeed();

    virtual ~RampGeneratorSpeed();

    void getParameter(quantity<si::angular_velocity>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::angular_velocity> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Actual sum of the I2t monitor.
///////////////////////////////////////////////////////////////////////////////
class I2tSum : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    I2tSum();

    virtual ~I2tSum();

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
/// Actual temperature of the motor driver. 
///////////////////////////////////////////////////////////////////////////////
class ActualMotorDriverTemperature : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ActualMotorDriverTemperature();

    virtual ~ActualMotorDriverTemperature();

    void getParameter(quantity<celsius::temperature>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<celsius::temperature> value;

    std::string name;

    ParameterType parameterType;

};
///////////////////////////////////////////////////////////////////////////////
/// Get actual supply current of the module.
///////////////////////////////////////////////////////////////////////////////
class ActualModuleSupplyCurrent : public YouBotJointParameterReadOnly {
friend class YouBotJoint;
  public:
    ActualModuleSupplyCurrent();

    virtual ~ActualModuleSupplyCurrent();

    void getParameter(quantity<si::current>& parameter) const;

    void toString(std::string& value);


  private:
    void getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const;

    void setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage);

    std::string getName() const {return this->name;};

    ParameterType getType() const {return this->parameterType;};

    quantity<si::current> value;

    std::string name;

    ParameterType parameterType;

};

} // namespace youbot
#endif
