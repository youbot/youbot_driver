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
#include "youbot/YouBotJointParameterReadOnly.hpp"
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

void ActualMotorVoltage::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0007E171
    parameter = this->value;
  // Bouml preserved body end 0007E171
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
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value; //TODO do convertion
    }
  // Bouml preserved body end 0007E271
}

ActualPWMDutyCycle::ActualPWMDutyCycle() {
  // Bouml preserved body begin 0007E3F1
    this->name = "ActualPWMDutyCycle";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007E3F1
}

ActualPWMDutyCycle::~ActualPWMDutyCycle() {
  // Bouml preserved body begin 0007E471
  // Bouml preserved body end 0007E471
}

void ActualPWMDutyCycle::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0007E4F1
    parameter = this->value;
  // Bouml preserved body end 0007E4F1
}

void ActualPWMDutyCycle::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007E571
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 153; //ActualPWMDutyCycle
 //   message.stctOutput.value = value;

  // Bouml preserved body end 0007E571
}

void ActualPWMDutyCycle::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007E5F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = (((double)message.stctInput.value)/3599.0)*100.0 ;
    }
  // Bouml preserved body end 0007E5F1
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
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
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
      LOG(error) << errorMessage << "got over current";
      //    throw JointErrorException(errorMessage + "got over current");
    }

    if (value & UNDER_VOLTAGE) {
      LOG(error) << errorMessage << "got under voltage";
      //    throw JointErrorException(errorMessage + "got under voltage");
    }

    if (value & OVER_VOLTAGE) {
      LOG(error) << errorMessage << "got over voltage";
      //   throw JointErrorException(errorMessage + "got over voltage");
    }

    if (value & OVER_TEMPERATURE) {
      LOG(error) << errorMessage << "got over temperature";
      //   throw JointErrorException(errorMessage + "got over temperature");
    }

    if (value & HALTED) {
      LOG(info) << errorMessage << "is halted";
      //   throw JointErrorException(errorMessage + "is halted");
    }

    if (value & HALL_SENSOR) {
      LOG(error) << errorMessage << "got hall sensor problem";
      //   throw JointErrorException(errorMessage + "got hall sensor problem");
    }

    if (value & ENCODER) {
      LOG(error) << errorMessage << "got encoder problem";
      //   throw JointErrorException(errorMessage + "got encoder problem");
    }

    if (value & MOTOR_WINDING) {
      LOG(error) << errorMessage << "got motor winding problem";
      //   throw JointErrorException(errorMessage + "got motor winding problem");
    }

    if (value & CYCLE_TIME_VIOLATION) {
      LOG(error) << errorMessage << "the cycle time is violated";
      //   throw JointErrorException(errorMessage + "the cycle time is violated");
    }

    if (value & INIT_SIN_COMM) {
      LOG(error) << errorMessage << "need to initialize the sinus commutation";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & POSITION_MODE) {
      LOG(info) << errorMessage << "position mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (value & POSITION_REACHED) {
      LOG(info) << errorMessage << "position reached";
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
    value = ((double) message.stctInput.value / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
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
    value = ((double) message.stctInput.value / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
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
    this->value = ((((double)message.stctInput.value) / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
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
    this->value = ((((double)message.stctInput.value) / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 000825F1
}


} // namespace youbot
