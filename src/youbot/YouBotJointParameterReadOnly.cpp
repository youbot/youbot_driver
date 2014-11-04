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
#include "youbot_driver/youbot/YouBotJointParameterReadOnly.hpp"
namespace youbot {

YouBotJointParameterReadOnly::YouBotJointParameterReadOnly() {
  // Bouml preserved body begin 0006FDF1
  // Bouml preserved body end 0006FDF1
}

YouBotJointParameterReadOnly::~YouBotJointParameterReadOnly() {
  // Bouml preserved body begin 0006FE71
  // Bouml preserved body end 0006FE71
}

ActualMotorVoltage::ActualMotorVoltage() {
  // Bouml preserved body begin 0007E071
    this->name = "ActualMotorVoltage";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007E071
}

ActualMotorVoltage::~ActualMotorVoltage() {
  // Bouml preserved body begin 0007E0F1
  // Bouml preserved body end 0007E0F1
}

void ActualMotorVoltage::getParameter(quantity<electric_potential>& parameter) const {
  // Bouml preserved body begin 0007E171
    parameter = this->value;
  // Bouml preserved body end 0007E171
}

void ActualMotorVoltage::toString(std::string& value) {
  // Bouml preserved body begin 0009EC71
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009EC71
}

void ActualMotorVoltage::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007E1F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 151; //ActualMotorVoltage
  //  message.stctOutput.value = value;

  // Bouml preserved body end 0007E1F1
}

void ActualMotorVoltage::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007E271
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      double temp = message.stctInput.value;
      this->value = temp/100.0 * volt; //TODO do convertion
    }
  // Bouml preserved body end 0007E271
}

ErrorAndStatus::ErrorAndStatus() {
  // Bouml preserved body begin 0007E771
    this->name = "ErrorAndStatus";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007E771
}

ErrorAndStatus::~ErrorAndStatus() {
  // Bouml preserved body begin 0007E7F1
  // Bouml preserved body end 0007E7F1
}

void ErrorAndStatus::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0007E871
    parameter = this->value;
    this->parseYouBotErrorFlags();
  // Bouml preserved body end 0007E871
}

void ErrorAndStatus::toString(std::string& value) {
  // Bouml preserved body begin 0009ED71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009ED71
}

void ErrorAndStatus::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007E8F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 156; //ErrorAndStatus
 //   message.stctOutput.value = value;

  // Bouml preserved body end 0007E8F1
}

void ErrorAndStatus::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007E971
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 0007E971
}

void ErrorAndStatus::parseYouBotErrorFlags() const {
  // Bouml preserved body begin 0007EAF1
   // std::stringstream errorMessageStream;
   // errorMessageStream << "Joint " << this->jointNumber << " ";
    std::string errorMessage;
   // errorMessage = errorMessageStream.str();


    if (value & OVER_CURRENT) {
      LOG(warning) << errorMessage << "over current";
      //    throw JointErrorException(errorMessage + "got over current");
    }

    if (value & UNDER_VOLTAGE) {
      LOG(warning) << errorMessage << "under voltage";
      //    throw JointErrorException(errorMessage + "got under voltage");
    }

    if (value & OVER_VOLTAGE) {
      LOG(warning) << errorMessage << "over voltage";
      //   throw JointErrorException(errorMessage + "got over voltage");
    }

    if (value & OVER_TEMPERATURE) {
      LOG(warning) << errorMessage << "over temperature";
      //   throw JointErrorException(errorMessage + "got over temperature");
    }

    if (value & MOTOR_HALTED) {
      LOG(info) << errorMessage << "is halted";
      //   throw JointErrorException(errorMessage + "is halted");
    }

    if (value & HALL_SENSOR_ERROR) {
      LOG(warning) << errorMessage << "hall sensor problem";
      //   throw JointErrorException(errorMessage + "got hall sensor problem");
    }

//    if (value & PWM_MODE_ACTIVE) {
//      LOG(info) << errorMessage << "PWM mode active";
      //   throw JointErrorException(errorMessage + "the cycle time is violated");
//    }

    if (value & VELOCITY_MODE) {
      LOG(info) << errorMessage << "velocity mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & POSITION_MODE) {
      LOG(info) << errorMessage << "position mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & TORQUE_MODE) {
      LOG(info) << errorMessage << "torque mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & POSITION_REACHED) {
      LOG(info) << errorMessage << "position reached";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & INITIALIZED) {
      LOG(info) << errorMessage << "is initialized";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & TIMEOUT) {
      LOG(warning) << errorMessage << "timeout";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & I2T_EXCEEDED) {
      LOG(warning) << errorMessage << "I2t exceeded";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

  // Bouml preserved body end 0007EAF1
}

PositionError::PositionError() {
  // Bouml preserved body begin 00081771
    this->name = "PositionError";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00081771
}

PositionError::~PositionError() {
  // Bouml preserved body begin 000817F1
  // Bouml preserved body end 000817F1
}

void PositionError::getParameter(quantity<plane_angle>& parameter) const {
  // Bouml preserved body begin 00081871
    parameter = this->value;
  // Bouml preserved body end 00081871
}

void PositionError::toString(std::string& value) {
  // Bouml preserved body begin 0009EDF1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009EDF1
}

void PositionError::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00081971
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 226; //PositionError
//    message.stctOutput.value = value;
  // Bouml preserved body end 00081971
}

void PositionError::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000819F1
    double temp = (int)message.stctInput.value;
    value = (temp / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
  // Bouml preserved body end 000819F1
}

PositionErrorSum::PositionErrorSum() {
  // Bouml preserved body begin 00081B71
    this->name = "PositionErrorSum";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00081B71
}

PositionErrorSum::~PositionErrorSum() {
  // Bouml preserved body begin 00081BF1
  // Bouml preserved body end 00081BF1
}

void PositionErrorSum::getParameter(quantity<plane_angle>& parameter) const {
  // Bouml preserved body begin 00081C71
    parameter = this->value;
  // Bouml preserved body end 00081C71
}

void PositionErrorSum::toString(std::string& value) {
  // Bouml preserved body begin 0009EE71
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009EE71
}

void PositionErrorSum::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00081D71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 227; //PositionErrorSum
 //   message.stctOutput.value = value;

  // Bouml preserved body end 00081D71
}

void PositionErrorSum::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00081DF1
    double temp = (int)message.stctInput.value;
    value = (temp / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
  // Bouml preserved body end 00081DF1
}

VelocityError::VelocityError() {
  // Bouml preserved body begin 00081F71
    this->name = "VelocityError";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00081F71
}

VelocityError::~VelocityError() {
  // Bouml preserved body begin 00081FF1
  // Bouml preserved body end 00081FF1
}

void VelocityError::getParameter(quantity<si::angular_velocity>& parameter) const {
  // Bouml preserved body begin 00082071
    parameter = this->value;
  // Bouml preserved body end 00082071
}

void VelocityError::toString(std::string& value) {
  // Bouml preserved body begin 0009EEF1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009EEF1
}

void VelocityError::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00082171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 228; //VelocityError
 //   message.stctOutput.value = value;

  // Bouml preserved body end 00082171
}

void VelocityError::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000821F1
    double temp = (int)message.stctInput.value;
    this->value = ((temp / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 000821F1
}

VelocityErrorSum::VelocityErrorSum() {
  // Bouml preserved body begin 00082371
    this->name = "VelocityErrorSum";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00082371
}

VelocityErrorSum::~VelocityErrorSum() {
  // Bouml preserved body begin 000823F1
  // Bouml preserved body end 000823F1
}

void VelocityErrorSum::getParameter(quantity<si::angular_velocity>& parameter) const {
  // Bouml preserved body begin 00082471
    parameter = this->value;
  // Bouml preserved body end 00082471
}

void VelocityErrorSum::toString(std::string& value) {
  // Bouml preserved body begin 0009EF71
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009EF71
}

void VelocityErrorSum::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00082571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 229; //VelocityErrorSum
 //   message.stctOutput.value = value;

  // Bouml preserved body end 00082571
}

void VelocityErrorSum::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000825F1
    double temp = (int32)message.stctInput.value;
    this->value = ((temp / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
    
  // Bouml preserved body end 000825F1
}

CurrentError::CurrentError() {
  // Bouml preserved body begin 000DAEF1
    this->name = "CurrentError";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000DAEF1
}

CurrentError::~CurrentError() {
  // Bouml preserved body begin 000DAF71
  // Bouml preserved body end 000DAF71
}

void CurrentError::getParameter(quantity<si::current>& parameter) const {
  // Bouml preserved body begin 000DAFF1
    parameter = this->value;
  // Bouml preserved body end 000DAFF1
}

void CurrentError::toString(std::string& value) {
  // Bouml preserved body begin 000DB071
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 000DB071
}

void CurrentError::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000DB0F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 200; //CurrentError
  //  message.stctOutput.value = value;

  // Bouml preserved body end 000DB0F1
}

void CurrentError::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000DB171
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      double temp = (int)message.stctInput.value;
      this->value = temp /1000.0 * ampere; //convert from milli A to A
    }
  // Bouml preserved body end 000DB171
}

CurrentErrorSum::CurrentErrorSum() {
  // Bouml preserved body begin 000DB2F1
    this->name = "CurrentErrorSum";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000DB2F1
}

CurrentErrorSum::~CurrentErrorSum() {
  // Bouml preserved body begin 000DB371
  // Bouml preserved body end 000DB371
}

void CurrentErrorSum::getParameter(quantity<si::current>& parameter) const {
  // Bouml preserved body begin 000DB3F1
    parameter = this->value;
  // Bouml preserved body end 000DB3F1
}

void CurrentErrorSum::toString(std::string& value) {
  // Bouml preserved body begin 000DB471
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 000DB471
}

void CurrentErrorSum::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000DB4F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 201; //CurrentErrorSum
  //  message.stctOutput.value = value;

  // Bouml preserved body end 000DB4F1
}

void CurrentErrorSum::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000DB571
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      double temp = (int)message.stctInput.value;
      this->value = temp /1000.0 * ampere; //convert from milli A to A
    }
  // Bouml preserved body end 000DB571
}

RampGeneratorSpeed::RampGeneratorSpeed() {
  // Bouml preserved body begin 0009F271
    this->name = "RampGeneratorSpeed";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0009F271
}

RampGeneratorSpeed::~RampGeneratorSpeed() {
  // Bouml preserved body begin 0009F2F1
  // Bouml preserved body end 0009F2F1
}

void RampGeneratorSpeed::getParameter(quantity<si::angular_velocity>& parameter) const {
  // Bouml preserved body begin 0009F371
    parameter = this->value;
  // Bouml preserved body end 0009F371
}

void RampGeneratorSpeed::toString(std::string& value) {
  // Bouml preserved body begin 0009F3F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009F3F1
}

void RampGeneratorSpeed::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0009F471

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 13; //RampGeneratorSpeed
 //   message.stctOutput.value = value;

  // Bouml preserved body end 0009F471
}

void RampGeneratorSpeed::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0009F4F1
    double temp = (int)message.stctInput.value;
    this->value = ((temp / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 0009F4F1
}

I2tSum::I2tSum() {
  // Bouml preserved body begin 000A0CF1
    this->name = "I2tSum";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000A0CF1
}

I2tSum::~I2tSum() {
  // Bouml preserved body begin 000A0D71
  // Bouml preserved body end 000A0D71
}

void I2tSum::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000A0DF1
    parameter = this->value;
  // Bouml preserved body end 000A0DF1
}

void I2tSum::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 000A0E71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000A0E71
}

void I2tSum::toString(std::string& value) {
  // Bouml preserved body begin 000A0EF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000A0EF1
}

void I2tSum::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000A0F71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 27; //I2tSum
    message.stctOutput.value = value;
  // Bouml preserved body end 000A0F71
}

void I2tSum::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A0FF1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 000A0FF1
}

ActualMotorDriverTemperature::ActualMotorDriverTemperature() {
  // Bouml preserved body begin 000CB071
    this->name = "ActualMotorDriverTemperature";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000CB071
}

ActualMotorDriverTemperature::~ActualMotorDriverTemperature() {
  // Bouml preserved body begin 000CB0F1
  // Bouml preserved body end 000CB0F1
}

void ActualMotorDriverTemperature::getParameter(quantity<celsius::temperature>& parameter) const {
  // Bouml preserved body begin 000CB171
    parameter = this->value;
  // Bouml preserved body end 000CB171
}

void ActualMotorDriverTemperature::toString(std::string& value) {
  // Bouml preserved body begin 000CB1F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 000CB1F1
}

void ActualMotorDriverTemperature::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000CB271
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 152; //ActualMotorDriverTemperature
  //  message.stctOutput.value = value;

  // Bouml preserved body end 000CB271
}

void ActualMotorDriverTemperature::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000CB2F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      double materialConstant = 3434;
      double R_NTC = ((double)9011.2/message.stctInput.value) - 2.2;
      double nominator = materialConstant * 298.16;
      double denominator = materialConstant + (log(R_NTC/10.0) * 298.16);
      this->value = ((nominator/denominator) - 273.16) * celsius::degree;
    }
  // Bouml preserved body end 000CB2F1
}

ActualModuleSupplyCurrent::ActualModuleSupplyCurrent() {
  // Bouml preserved body begin 000CB471
    this->name = "ActualModuleSupplyCurrent";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000CB471
}

ActualModuleSupplyCurrent::~ActualModuleSupplyCurrent() {
  // Bouml preserved body begin 000CB4F1
  // Bouml preserved body end 000CB4F1
}

void ActualModuleSupplyCurrent::getParameter(quantity<si::current>& parameter) const {
  // Bouml preserved body begin 000CB571
    parameter = this->value;
  // Bouml preserved body end 000CB571
}

void ActualModuleSupplyCurrent::toString(std::string& value) {
  // Bouml preserved body begin 000CB5F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 000CB5F1
}

void ActualModuleSupplyCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000CB671
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 157; //ActualModuleSupplyCurrent
  //  message.stctOutput.value = value;

  // Bouml preserved body end 000CB671
}

void ActualModuleSupplyCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000CB6F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (double)message.stctInput.value /1000.0 * ampere; //convert from milli A to A
    }
  // Bouml preserved body end 000CB6F1
}


} // namespace youbot
