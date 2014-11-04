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
#include "youbot_driver/youbot/YouBotJointParameterPasswordProtected.hpp"
namespace youbot {

YouBotJointParameterPasswordProtected::YouBotJointParameterPasswordProtected() {
  // Bouml preserved body begin 000A4CF1
  // Bouml preserved body end 000A4CF1
}

YouBotJointParameterPasswordProtected::~YouBotJointParameterPasswordProtected() {
  // Bouml preserved body begin 000A4D71
  // Bouml preserved body end 000A4D71
}

ActivateOvervoltageProtection::ActivateOvervoltageProtection() {
  // Bouml preserved body begin 00083371
    this->name = "ActivateOvervoltageProtection";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00083371
}

ActivateOvervoltageProtection::~ActivateOvervoltageProtection() {
  // Bouml preserved body begin 000833F1
  // Bouml preserved body end 000833F1
}

void ActivateOvervoltageProtection::getParameter(bool& parameter) const {
  // Bouml preserved body begin 00083471
    parameter = this->value;
  // Bouml preserved body end 00083471
}

void ActivateOvervoltageProtection::setParameter(const bool parameter) {
  // Bouml preserved body begin 000834F1
    this->value = parameter;
  // Bouml preserved body end 000834F1
}

void ActivateOvervoltageProtection::toString(std::string& value) {
  // Bouml preserved body begin 0009E4F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E4F1
}

void ActivateOvervoltageProtection::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00083571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 245; //ActivateOvervoltageProtection
    message.stctOutput.value = value;

  // Bouml preserved body end 00083571
}

void ActivateOvervoltageProtection::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000835F1
    this->value = message.stctInput.value; 
  // Bouml preserved body end 000835F1
}

ActualCommutationOffset::ActualCommutationOffset() {
  // Bouml preserved body begin 0007D871
    this->name = "ActualCommutationOffset";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007D871
}

ActualCommutationOffset::~ActualCommutationOffset() {
  // Bouml preserved body begin 0007D8F1
  // Bouml preserved body end 0007D8F1
}

void ActualCommutationOffset::getParameter(int& parameter) const {
  // Bouml preserved body begin 0007D971
    parameter = this->value;
  // Bouml preserved body end 0007D971
}

void ActualCommutationOffset::setParameter(const int parameter) {
  // Bouml preserved body begin 0007D9F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007D9F1
}

void ActualCommutationOffset::toString(std::string& value) {
  // Bouml preserved body begin 0009D771
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D771
}

void ActualCommutationOffset::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007DA71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 165; //ActualCommutationOffset
    message.stctOutput.value = value;

  // Bouml preserved body end 0007DA71
}

void ActualCommutationOffset::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007DAF1
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 0007DAF1
}

ApproveProtectedParameters::ApproveProtectedParameters() {
  // Bouml preserved body begin 000956F1
    this->name = "ApproveProtectedParameters";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000956F1
}

ApproveProtectedParameters::~ApproveProtectedParameters() {
  // Bouml preserved body begin 00095771
  // Bouml preserved body end 00095771
}

void ApproveProtectedParameters::getParameter(int& parameter) const {
  // Bouml preserved body begin 000957F1
    parameter = this->value;
  // Bouml preserved body end 000957F1
}

void ApproveProtectedParameters::setParameter(const int parameter) {
  // Bouml preserved body begin 00095871
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 00095871
}

void ApproveProtectedParameters::toString(std::string& value) {
  // Bouml preserved body begin 0009E671
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E671
}

void ApproveProtectedParameters::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000958F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 248; //ApproveProtectedParameters
    message.stctOutput.value = value;
  // Bouml preserved body end 000958F1
}

void ApproveProtectedParameters::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00095971
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 00095971
}

BEMFConstant::BEMFConstant() {
  // Bouml preserved body begin 00082B71
    this->name = "BEMFConstant";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00082B71
}

BEMFConstant::~BEMFConstant() {
  // Bouml preserved body begin 00082BF1
  // Bouml preserved body end 00082BF1
}

void BEMFConstant::getParameter(int& parameter) const {
  // Bouml preserved body begin 00082C71
    parameter = this->value;
  // Bouml preserved body end 00082C71
}

void BEMFConstant::setParameter(const int parameter) {
  // Bouml preserved body begin 00082CF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00082CF1
}

void BEMFConstant::toString(std::string& value) {
  // Bouml preserved body begin 0009E1F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E1F1
}

void BEMFConstant::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00082D71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 239; //BEMFConstant
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 00082D71
}

void BEMFConstant::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00082DF1
    this->value = (int32)message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 00082DF1
}

CommutationMode::CommutationMode() {
  // Bouml preserved body begin 000704F1
    this->name = "CommutationMode";
    this->lowerLimit = 0;
    this->upperLimit = 5;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000704F1
}

CommutationMode::~CommutationMode() {
  // Bouml preserved body begin 00070571
  // Bouml preserved body end 00070571
}

void CommutationMode::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000705F1
    parameter = this->value;
  // Bouml preserved body end 000705F1
}

void CommutationMode::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 00093471
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 00093471
}

void CommutationMode::toString(std::string& value) {
  // Bouml preserved body begin 0009E7F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E7F1
}

void CommutationMode::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00070671
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 159; //CommutationMode
    message.stctOutput.value = value;
  // Bouml preserved body end 00070671
}

void CommutationMode::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000720F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = message.stctInput.value; //TODO do convertion
    }
  // Bouml preserved body end 000720F1
}

CommutationMotorCurrent::CommutationMotorCurrent() {
  // Bouml preserved body begin 0008C371
    this->name = "CommutationMotorCurrent";
    this->lowerLimit = 0 * ampere;
    this->upperLimit = INT_MAX * ampere;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0008C371
}

CommutationMotorCurrent::~CommutationMotorCurrent() {
  // Bouml preserved body begin 0008C3F1
  // Bouml preserved body end 0008C3F1
}

void CommutationMotorCurrent::getParameter(quantity<current>& parameter) const {
  // Bouml preserved body begin 0008C471
    parameter = this->value;
  // Bouml preserved body end 0008C471
}

void CommutationMotorCurrent::setParameter(const quantity<current>& parameter) {
  // Bouml preserved body begin 0008C4F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0008C4F1
}

void CommutationMotorCurrent::toString(std::string& value) {
  // Bouml preserved body begin 0009DCF1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009DCF1
}

void CommutationMotorCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0008C571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 177; //CommutationMotorCurrent
    message.stctOutput.value = (uint32)(value.value() * 1000.0); // ampere to milli ampere

  // Bouml preserved body end 0008C571
}

void CommutationMotorCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0008C5F1
  double temp = (uint32)message.stctInput.value;
  this->value = temp/1000.0 * ampere; //milli ampere to ampere
  // Bouml preserved body end 0008C5F1
}

CurrentControlLoopDelay::CurrentControlLoopDelay() {
  // Bouml preserved body begin 00079F71
    this->name = "CurrentControlLoopDelay";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX * seconds;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00079F71
}

CurrentControlLoopDelay::~CurrentControlLoopDelay() {
  // Bouml preserved body begin 00079FF1
  // Bouml preserved body end 00079FF1
}

void CurrentControlLoopDelay::getParameter(quantity<si::time>& parameter) const {
  // Bouml preserved body begin 0007A071
    parameter = this->value;
  // Bouml preserved body end 0007A071
}

void CurrentControlLoopDelay::setParameter(const quantity<si::time>& parameter) {
  // Bouml preserved body begin 0007A0F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007A0F1
}

void CurrentControlLoopDelay::toString(std::string& value) {
  // Bouml preserved body begin 0009CFF1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009CFF1
}

void CurrentControlLoopDelay::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007A171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 134; //CurrentControlLoopDelay
    message.stctOutput.value = (uint32)(value.value() * 1000 *1000 /50.0); //sec to µsec

  // Bouml preserved body end 0007A171
}

void CurrentControlLoopDelay::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007A1F1
    double temp = (uint32)message.stctInput.value;
    this->value = (temp/(1000.0 * 1000.0)) * 50 * seconds; //µsec to sec
  // Bouml preserved body end 0007A1F1
}

EncoderResolution::EncoderResolution() {
  // Bouml preserved body begin 000713F1
    this->name = "EncoderResolution";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000713F1
}

EncoderResolution::~EncoderResolution() {
  // Bouml preserved body begin 00071471
  // Bouml preserved body end 00071471
}

void EncoderResolution::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000714F1
    parameter = this->value;
  // Bouml preserved body end 000714F1
}

void EncoderResolution::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 00093671
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 00093671
}

void EncoderResolution::toString(std::string& value) {
  // Bouml preserved body begin 0009E9F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E9F1
}

void EncoderResolution::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00071571
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 250; //EncoderResolution
    message.stctOutput.value = (uint32)value;
  // Bouml preserved body end 00071571
}

void EncoderResolution::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072371
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (uint32)message.stctInput.value;
    }
  // Bouml preserved body end 00072371
}

EncoderStopSwitch::EncoderStopSwitch() {
  // Bouml preserved body begin 0007D471
    this->name = "EncoderStopSwitch";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007D471
}

EncoderStopSwitch::~EncoderStopSwitch() {
  // Bouml preserved body begin 0007D4F1
  // Bouml preserved body end 0007D4F1
}

void EncoderStopSwitch::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0007D571
    parameter = this->value;
  // Bouml preserved body end 0007D571
}

void EncoderStopSwitch::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 0007D5F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007D5F1
}

void EncoderStopSwitch::toString(std::string& value) {
  // Bouml preserved body begin 0009D6F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D6F1
}

void EncoderStopSwitch::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007D671

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 164; //EncoderStopSwitch
    message.stctOutput.value = (uint32)value;

  // Bouml preserved body end 0007D671
}

void EncoderStopSwitch::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007D6F1
    this->value = (uint32)message.stctInput.value;
  // Bouml preserved body end 0007D6F1
}

HallSensorPolarityReversal::HallSensorPolarityReversal() {
  // Bouml preserved body begin 00071CF1
    this->name = "HallSensorPolarityReversal";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00071CF1
}

HallSensorPolarityReversal::~HallSensorPolarityReversal() {
  // Bouml preserved body begin 00071D71
  // Bouml preserved body end 00071D71
}

void HallSensorPolarityReversal::getParameter(bool& parameter) const {
  // Bouml preserved body begin 00071DF1
    parameter = this->value;
  // Bouml preserved body end 00071DF1
}

void HallSensorPolarityReversal::setParameter(const bool parameter) {
  // Bouml preserved body begin 00093771
    this->value = parameter;
  // Bouml preserved body end 00093771
}

void HallSensorPolarityReversal::toString(std::string& value) {
  // Bouml preserved body begin 0009EAF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009EAF1
}

void HallSensorPolarityReversal::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00071E71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 254; //HallSensorPolarityReversal
    message.stctOutput.value = (uint32)value;
  // Bouml preserved body end 00071E71
}

void HallSensorPolarityReversal::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000724F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (uint32)message.stctInput.value;
    }
  // Bouml preserved body end 000724F1
}

I2tExceedCounter::I2tExceedCounter() {
  // Bouml preserved body begin 000A1171
    this->name = "I2tExceedCounter";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000A1171
}

I2tExceedCounter::~I2tExceedCounter() {
  // Bouml preserved body begin 000A11F1
  // Bouml preserved body end 000A11F1
}

void I2tExceedCounter::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000A1271
    parameter = this->value;
  // Bouml preserved body end 000A1271
}

void I2tExceedCounter::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 000A12F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000A12F1
}

void I2tExceedCounter::toString(std::string& value) {
  // Bouml preserved body begin 000A1371
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000A1371
}

void I2tExceedCounter::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000A13F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 28; //I2tExceedCounter
    message.stctOutput.value = (uint32)value;
  // Bouml preserved body end 000A13F1
}

void I2tExceedCounter::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A1471
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (uint32)message.stctInput.value;
    }
  // Bouml preserved body end 000A1471
}

I2tLimit::I2tLimit() {
  // Bouml preserved body begin 000A0871
    this->name = "I2tLimit";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000A0871
}

I2tLimit::~I2tLimit() {
  // Bouml preserved body begin 000A08F1
  // Bouml preserved body end 000A08F1
}

void I2tLimit::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000A0971
    parameter = this->value;
  // Bouml preserved body end 000A0971
}

void I2tLimit::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 000A09F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000A09F1
}

void I2tLimit::toString(std::string& value) {
  // Bouml preserved body begin 000A0A71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000A0A71
}

void I2tLimit::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000A0AF1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 26; //I2tLimit
    message.stctOutput.value = (uint32)value;
  // Bouml preserved body end 000A0AF1
}

void I2tLimit::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A0B71
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (uint32)message.stctInput.value;
    }
  // Bouml preserved body end 000A0B71
}

InitializationMode::InitializationMode() {
  // Bouml preserved body begin 000710F1
    this->name = "InitializationMode";
    this->lowerLimit = 0;
    this->upperLimit = 2;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000710F1
}

InitializationMode::~InitializationMode() {
  // Bouml preserved body begin 00071171
  // Bouml preserved body end 00071171
}

void InitializationMode::getParameter(int& parameter) const {
  // Bouml preserved body begin 000711F1
    parameter = this->value;
  // Bouml preserved body end 000711F1
}

void InitializationMode::setParameter(const int parameter) {
  // Bouml preserved body begin 000935F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000935F1
}

void InitializationMode::toString(std::string& value) {
  // Bouml preserved body begin 0009E971
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E971
}

void InitializationMode::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00071271
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 249; //InitializationMode
    message.stctOutput.value = value;
  // Bouml preserved body end 00071271
}

void InitializationMode::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000722F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (int32)message.stctInput.value; 
    }
  // Bouml preserved body end 000722F1
}

InitSineDelay::InitSineDelay() {
  // Bouml preserved body begin 00082F71
    this->name = "InitSineDelay";
    this->lowerLimit = -32.768 * seconds;
    this->upperLimit = +32.767 * seconds;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00082F71
}

InitSineDelay::~InitSineDelay() {
  // Bouml preserved body begin 00082FF1
  // Bouml preserved body end 00082FF1
}

void InitSineDelay::getParameter(quantity<si::time>& parameter) const {
  // Bouml preserved body begin 00083071
    parameter = this->value;
  // Bouml preserved body end 00083071
}

void InitSineDelay::setParameter(const quantity<si::time>& parameter) {
  // Bouml preserved body begin 000830F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000830F1
}

void InitSineDelay::toString(std::string& value) {
  // Bouml preserved body begin 0009E3F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009E3F1
}

void InitSineDelay::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00083171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 244; //InitSineDelay
    message.stctOutput.value = (int32)(value.value() * 1000); //sec to µsec

  // Bouml preserved body end 00083171
}

void InitSineDelay::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000831F1
  double temp = (int32)message.stctInput.value;
  this->value = (temp/1000.0)  * seconds; //µsec to sec
  // Bouml preserved body end 000831F1
}

MassInertiaConstant::MassInertiaConstant() {
  // Bouml preserved body begin 00082771
    this->name = "MassInertiaConstant";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00082771
}

MassInertiaConstant::~MassInertiaConstant() {
  // Bouml preserved body begin 000827F1
  // Bouml preserved body end 000827F1
}

void MassInertiaConstant::getParameter(int& parameter) const {
  // Bouml preserved body begin 00082871
    parameter = this->value;
  // Bouml preserved body end 00082871
}

void MassInertiaConstant::setParameter(const int parameter) {
  // Bouml preserved body begin 000828F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000828F1
}

void MassInertiaConstant::toString(std::string& value) {
  // Bouml preserved body begin 0009E171
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E171
}

void MassInertiaConstant::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00082971

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 238; //MassInertiaConstant
    message.stctOutput.value = (int32)value; //TODO do convertion

  // Bouml preserved body end 00082971
}

void MassInertiaConstant::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000829F1
    this->value = (int32)message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 000829F1
}

MaximumMotorCurrent::MaximumMotorCurrent() {
  // Bouml preserved body begin 0006A5F1
    this->name = "MaximumMotorCurrent";
    this->lowerLimit = 0 * ampere;
    this->upperLimit = INT_MAX * ampere;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006A5F1
}

MaximumMotorCurrent::~MaximumMotorCurrent() {
  // Bouml preserved body begin 0006A671
  // Bouml preserved body end 0006A671
}

void MaximumMotorCurrent::getParameter(quantity<current>& parameter) const {
  // Bouml preserved body begin 0006A6F1
    parameter = this->value;
  // Bouml preserved body end 0006A6F1
}

void MaximumMotorCurrent::setParameter(const quantity<current>& parameter) {
  // Bouml preserved body begin 0006A771
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006A771
}

void MaximumMotorCurrent::toString(std::string& value) {
  // Bouml preserved body begin 0009CA71
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009CA71
}

void MaximumMotorCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006A7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 6; //MaximumMotorCurrent
    message.stctOutput.value = (uint32)(value.value() * 1000.0); // ampere to milli ampere

  // Bouml preserved body end 0006A7F1
}

void MaximumMotorCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006A871
  double temp = (uint32)message.stctInput.value;
  this->value = temp/1000.0 * ampere; //milli ampere to ampere
  // Bouml preserved body end 0006A871
}

MotorCoilResistance::MotorCoilResistance() {
  // Bouml preserved body begin 00070DF1
    this->name = "MotorCoilResistance";
    this->lowerLimit = INT_MIN * ohm;
    this->upperLimit = INT_MAX * ohm;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00070DF1
}

MotorCoilResistance::~MotorCoilResistance() {
  // Bouml preserved body begin 00070E71
  // Bouml preserved body end 00070E71
}

void MotorCoilResistance::getParameter(quantity<resistance>& parameter) const {
  // Bouml preserved body begin 00070EF1
    parameter = this->value;
  // Bouml preserved body end 00070EF1
}

void MotorCoilResistance::setParameter(const quantity<resistance>& parameter) {
  // Bouml preserved body begin 00093571
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 00093571
}

void MotorCoilResistance::toString(std::string& value) {
  // Bouml preserved body begin 0009E8F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009E8F1
}

void MotorCoilResistance::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00070F71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 240; //MotorCoilResistance
    message.stctOutput.value = (int32)(value.value() * 1000); //from ohm to milli ohm
  // Bouml preserved body end 00070F71
}

void MotorCoilResistance::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072271
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      double temp = (int32)message.stctInput.value;
      this->value = temp/1000.0 * ohm;
    }
  // Bouml preserved body end 00072271
}

MotorControllerTimeout::MotorControllerTimeout() {
  // Bouml preserved body begin 0009F671
    this->name = "MotorControllerTimeout";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX * seconds;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0009F671
}

MotorControllerTimeout::~MotorControllerTimeout() {
  // Bouml preserved body begin 0009F6F1
  // Bouml preserved body end 0009F6F1
}

void MotorControllerTimeout::getParameter(quantity<si::time>& parameter) const {
  // Bouml preserved body begin 0009F771
    parameter = this->value;
  // Bouml preserved body end 0009F771
}

void MotorControllerTimeout::setParameter(const quantity<si::time>& parameter) {
  // Bouml preserved body begin 0009F7F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0009F7F1
}

void MotorControllerTimeout::toString(std::string& value) {
  // Bouml preserved body begin 0009F871
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009F871
}

void MotorControllerTimeout::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0009F8F1


    if (msgType == SAP){
      message.stctOutput.commandNumber = SGP;
    }else if (msgType == GAP) {
      message.stctOutput.commandNumber = GGP;
    } else {
      message.stctOutput.commandNumber = msgType;
    }
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 90; //MotorControllerTimeout
    message.stctOutput.value = value.value() * 1000; //sec to milli sec

  // Bouml preserved body end 0009F8F1
}

void MotorControllerTimeout::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0009F971
    this->value = ((double)message.stctInput.value)/1000.0 * seconds; //milli sec to sec
  // Bouml preserved body end 0009F971
}

MotorPoles::MotorPoles() {
  // Bouml preserved body begin 000719F1
    this->name = "MotorPoles";
    this->lowerLimit = 2;
    this->upperLimit = 254;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000719F1
}

MotorPoles::~MotorPoles() {
  // Bouml preserved body begin 00071A71
  // Bouml preserved body end 00071A71
}

void MotorPoles::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 00071AF1
    parameter = this->value;
  // Bouml preserved body end 00071AF1
}

void MotorPoles::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 000937F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000937F1
}

void MotorPoles::toString(std::string& value) {
  // Bouml preserved body begin 0009EB71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009EB71
}

void MotorPoles::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00071B71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 253; //MotorPoles
    message.stctOutput.value = (uint32)value;
  // Bouml preserved body end 00071B71
}

void MotorPoles::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072471
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = (uint32)message.stctInput.value;
    }
  // Bouml preserved body end 00072471
}

OperationalTime::OperationalTime() {
  // Bouml preserved body begin 000A03F1
    this->name = "OperationalTime";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX * seconds;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000A03F1
}

OperationalTime::~OperationalTime() {
  // Bouml preserved body begin 000A0471
  // Bouml preserved body end 000A0471
}

void OperationalTime::getParameter(quantity<si::time>& parameter) const {
  // Bouml preserved body begin 000A04F1
    parameter = this->value;
  // Bouml preserved body end 000A04F1
}

void OperationalTime::setParameter(const quantity<si::time>& parameter) {
  // Bouml preserved body begin 000A0571
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000A0571
}

void OperationalTime::toString(std::string& value) {
  // Bouml preserved body begin 000A05F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 000A05F1
}

void OperationalTime::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000A0671
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 30; //OperationalTime
    message.stctOutput.value = value.value() / 60.0; //sec to min

  // Bouml preserved body end 000A0671
}

void OperationalTime::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A06F1
    this->value = ((double)message.stctInput.value) * 60.0 * seconds; //min to sec
  // Bouml preserved body end 000A06F1
}

PIDControlTime::PIDControlTime() {
  // Bouml preserved body begin 0006ADF1
    this->name = "PIDControlTime";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX * seconds;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006ADF1
}

PIDControlTime::~PIDControlTime() {
  // Bouml preserved body begin 0006AE71
  // Bouml preserved body end 0006AE71
}

void PIDControlTime::getParameter(quantity<si::time>& parameter) const {
  // Bouml preserved body begin 0006AEF1
    parameter = this->value;
  // Bouml preserved body end 0006AEF1
}

void PIDControlTime::setParameter(const quantity<si::time>& parameter) {
  // Bouml preserved body begin 0006AF71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006AF71
}

void PIDControlTime::toString(std::string& value) {
  // Bouml preserved body begin 0009CF71
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009CF71
}

void PIDControlTime::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006AFF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 133; //PIDControlTime
    message.stctOutput.value = value.value() * 1000; //sec to milli sec

  // Bouml preserved body end 0006AFF1
}

void PIDControlTime::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006B071
    this->value = ((double)message.stctInput.value)/1000 * seconds; //milli sec to sec
  // Bouml preserved body end 0006B071
}

ReversingEncoderDirection::ReversingEncoderDirection() {
  // Bouml preserved body begin 000716F1
    this->name = "ReversingEncoderDirection";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000716F1
}

ReversingEncoderDirection::~ReversingEncoderDirection() {
  // Bouml preserved body begin 00071771
  // Bouml preserved body end 00071771
}

bool ReversingEncoderDirection::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000717F1
    parameter = this->value;
    return this->value;
  // Bouml preserved body end 000717F1
}

void ReversingEncoderDirection::setParameter(const bool parameter) {
  // Bouml preserved body begin 000936F1
    this->value = parameter;
  // Bouml preserved body end 000936F1
}

void ReversingEncoderDirection::toString(std::string& value) {
  // Bouml preserved body begin 0009EA71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009EA71
}

void ReversingEncoderDirection::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00071871
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 251; //ReversingEncoderDirection
    message.stctOutput.value = value;
  // Bouml preserved body end 00071871
}

void ReversingEncoderDirection::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000723F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == MAILBOX_SUCCESS) {
      this->value = message.stctInput.value; 
    }
  // Bouml preserved body end 000723F1
}

SetEncoderCounterZeroAtNextNChannel::SetEncoderCounterZeroAtNextNChannel() {
  // Bouml preserved body begin 0007C871
    this->name = "SetEncoderCounterZeroAtNextNChannel";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007C871
}

SetEncoderCounterZeroAtNextNChannel::~SetEncoderCounterZeroAtNextNChannel() {
  // Bouml preserved body begin 0007C8F1
  // Bouml preserved body end 0007C8F1
}

void SetEncoderCounterZeroAtNextNChannel::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0007C971
    parameter = this->value;
  // Bouml preserved body end 0007C971
}

void SetEncoderCounterZeroAtNextNChannel::setParameter(const bool parameter) {
  // Bouml preserved body begin 0007C9F1
    this->value = parameter;
  // Bouml preserved body end 0007C9F1
}

void SetEncoderCounterZeroAtNextNChannel::toString(std::string& value) {
  // Bouml preserved body begin 0009D571
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D571
}

void SetEncoderCounterZeroAtNextNChannel::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007CA71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 161; //SetEncoderCounterZeroAtNextNChannel
    message.stctOutput.value = value;

  // Bouml preserved body end 0007CA71
}

void SetEncoderCounterZeroAtNextNChannel::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007CAF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007CAF1
}

SetEncoderCounterZeroAtNextSwitch::SetEncoderCounterZeroAtNextSwitch() {
  // Bouml preserved body begin 0007CC71
    this->name = "SetEncoderCounterZeroAtNextSwitch";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007CC71
}

SetEncoderCounterZeroAtNextSwitch::~SetEncoderCounterZeroAtNextSwitch() {
  // Bouml preserved body begin 0007CCF1
  // Bouml preserved body end 0007CCF1
}

void SetEncoderCounterZeroAtNextSwitch::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0007CD71
    parameter = this->value;
  // Bouml preserved body end 0007CD71
}

void SetEncoderCounterZeroAtNextSwitch::setParameter(const bool parameter) {
  // Bouml preserved body begin 0007CDF1
    this->value = parameter;
  // Bouml preserved body end 0007CDF1
}

void SetEncoderCounterZeroAtNextSwitch::toString(std::string& value) {
  // Bouml preserved body begin 0009D5F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D5F1
}

void SetEncoderCounterZeroAtNextSwitch::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007CE71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 162; //SetEncoderCounterZeroAtNextSwitch
    message.stctOutput.value = value;

  // Bouml preserved body end 0007CE71
}

void SetEncoderCounterZeroAtNextSwitch::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007CEF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007CEF1
}

SetEncoderCounterZeroOnlyOnce::SetEncoderCounterZeroOnlyOnce() {
  // Bouml preserved body begin 0007D071
    this->name = "SetEncoderCounterZeroOnlyOnce";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007D071
}

SetEncoderCounterZeroOnlyOnce::~SetEncoderCounterZeroOnlyOnce() {
  // Bouml preserved body begin 0007D0F1
  // Bouml preserved body end 0007D0F1
}

void SetEncoderCounterZeroOnlyOnce::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0007D171
    parameter = this->value;
  // Bouml preserved body end 0007D171
}

void SetEncoderCounterZeroOnlyOnce::setParameter(const bool parameter) {
  // Bouml preserved body begin 0007D1F1
    this->value = parameter;
  // Bouml preserved body end 0007D1F1
}

void SetEncoderCounterZeroOnlyOnce::toString(std::string& value) {
  // Bouml preserved body begin 0009D671
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D671
}

void SetEncoderCounterZeroOnlyOnce::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007D271

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 163; //SetEncoderCounterZeroOnlyOnce
    message.stctOutput.value = value;

  // Bouml preserved body end 0007D271
}

void SetEncoderCounterZeroOnlyOnce::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007D2F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007D2F1
}

SineInitializationVelocity::SineInitializationVelocity() {
  // Bouml preserved body begin 0006EDF1
    this->name = "SineInitializationVelocity";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006EDF1
}

SineInitializationVelocity::~SineInitializationVelocity() {
  // Bouml preserved body begin 0006EE71
  // Bouml preserved body end 0006EE71
}

void SineInitializationVelocity::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006EEF1
    parameter = this->value;
  // Bouml preserved body end 0006EEF1
}

void SineInitializationVelocity::setParameter(const int parameter) {
  // Bouml preserved body begin 0006EF71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006EF71
}

void SineInitializationVelocity::toString(std::string& value) {
  // Bouml preserved body begin 0009E271
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E271
}

void SineInitializationVelocity::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006EFF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 241; //SineInitializationVelocity
    message.stctOutput.value = (int32)value;
  // Bouml preserved body end 0006EFF1
}

void SineInitializationVelocity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006F071
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 0006F071
}

StopSwitchPolarity::StopSwitchPolarity() {
  // Bouml preserved body begin 0007DC71
    this->name = "StopSwitchPolarity";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007DC71
}

StopSwitchPolarity::~StopSwitchPolarity() {
  // Bouml preserved body begin 0007DCF1
  // Bouml preserved body end 0007DCF1
}

void StopSwitchPolarity::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0007DD71
    parameter = this->value;
  // Bouml preserved body end 0007DD71
}

void StopSwitchPolarity::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 0007DDF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007DDF1
}

void StopSwitchPolarity::toString(std::string& value) {
  // Bouml preserved body begin 0009D7F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D7F1
}

void StopSwitchPolarity::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007DE71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 166; //StopSwitchPolarity
    message.stctOutput.value = value; 

  // Bouml preserved body end 0007DE71
}

void StopSwitchPolarity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007DEF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007DEF1
}

ThermalWindingTimeConstant::ThermalWindingTimeConstant() {
  // Bouml preserved body begin 0009FF71
    this->name = "ThermalWindingTimeConstant";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX * seconds;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0009FF71
}

ThermalWindingTimeConstant::~ThermalWindingTimeConstant() {
  // Bouml preserved body begin 0009FFF1
  // Bouml preserved body end 0009FFF1
}

void ThermalWindingTimeConstant::getParameter(quantity<si::time>& parameter) const {
  // Bouml preserved body begin 000A0071
    parameter = this->value;
  // Bouml preserved body end 000A0071
}

void ThermalWindingTimeConstant::setParameter(const quantity<si::time>& parameter) {
  // Bouml preserved body begin 000A00F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000A00F1
}

void ThermalWindingTimeConstant::toString(std::string& value) {
  // Bouml preserved body begin 000A0171
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 000A0171
}

void ThermalWindingTimeConstant::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000A01F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 25; //ThermalWindingTimeConstant
    message.stctOutput.value = value.value() * 1000; //sec to milli sec

  // Bouml preserved body end 000A01F1
}

void ThermalWindingTimeConstant::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A0271
    this->value = ((double)message.stctInput.value)/1000.0 * seconds; //milli sec to sec
  // Bouml preserved body end 000A0271
}

MotorHaltedVelocity::MotorHaltedVelocity() {
  // Bouml preserved body begin 000CB871
    this->name = "MotorHaltedVelocity";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000CB871
}

MotorHaltedVelocity::~MotorHaltedVelocity() {
  // Bouml preserved body begin 000CB8F1
  // Bouml preserved body end 000CB8F1
}

void MotorHaltedVelocity::getParameter(int& parameter) const {
  // Bouml preserved body begin 000CB971
    parameter = this->value;
  // Bouml preserved body end 000CB971
}

void MotorHaltedVelocity::setParameter(const int parameter) {
  // Bouml preserved body begin 000CB9F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000CB9F1
}

void MotorHaltedVelocity::toString(std::string& value) {
  // Bouml preserved body begin 000CBA71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000CBA71
}

void MotorHaltedVelocity::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000CBAF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 9; //MotorHaltedVelocity
    message.stctOutput.value = (int32)value;
  // Bouml preserved body end 000CBAF1
}

void MotorHaltedVelocity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000CBB71
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 000CBB71
}


} // namespace youbot
