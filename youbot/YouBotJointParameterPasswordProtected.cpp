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
#include "youbot/YouBotJointParameterPasswordProtected.hpp"
namespace youbot {

YouBotJointParameterPasswordProtected::YouBotJointParameterPasswordProtected() {
  // Bouml preserved body begin 000A4CF1
  // Bouml preserved body end 000A4CF1
}

YouBotJointParameterPasswordProtected::~YouBotJointParameterPasswordProtected() {
  // Bouml preserved body begin 000A4D71
  // Bouml preserved body end 000A4D71
}

void YouBotJointParameterPasswordProtected::toString(std::string& value) {
  // Bouml preserved body begin 000A4DF1
  // Bouml preserved body end 000A4DF1
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
    this->value = message.stctInput.value;
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
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
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
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 00082DF1
}

ClearI2tExceededFlag::ClearI2tExceededFlag() {
  // Bouml preserved body begin 000A15F1
    this->name = "ClearI2tExceededFlag";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
    this->value = true;
  // Bouml preserved body end 000A15F1
}

ClearI2tExceededFlag::~ClearI2tExceededFlag() {
  // Bouml preserved body begin 000A1671
  // Bouml preserved body end 000A1671
}

void ClearI2tExceededFlag::getParameter() const {
  // Bouml preserved body begin 000A16F1
  // Bouml preserved body end 000A16F1
}

void ClearI2tExceededFlag::setParameter() {
  // Bouml preserved body begin 000A1771
  // Bouml preserved body end 000A1771
}

void ClearI2tExceededFlag::toString(std::string& value) {
  // Bouml preserved body begin 000A17F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000A17F1
}

void ClearI2tExceededFlag::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000A1871
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 29; //ClearI2tExceededFlag
    message.stctOutput.value = value;
  // Bouml preserved body end 000A1871
}

void ClearI2tExceededFlag::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A18F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 000A18F1
}

ClearISumIfOvershootsTarget::ClearISumIfOvershootsTarget() {
  // Bouml preserved body begin 0007AB71
    this->name = "ClearISumIfOvershootsTarget";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007AB71
}

ClearISumIfOvershootsTarget::~ClearISumIfOvershootsTarget() {
  // Bouml preserved body begin 0007ABF1
  // Bouml preserved body end 0007ABF1
}

void ClearISumIfOvershootsTarget::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0007AC71
    parameter = this->value;
  // Bouml preserved body end 0007AC71
}

void ClearISumIfOvershootsTarget::setParameter(const bool parameter) {
  // Bouml preserved body begin 0007ACF1
    this->value = parameter;
  // Bouml preserved body end 0007ACF1
}

void ClearISumIfOvershootsTarget::toString(std::string& value) {
  // Bouml preserved body begin 0009D1F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D1F1
}

void ClearISumIfOvershootsTarget::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007AD71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 138; //ClearISumIfOvershootsTarget
    message.stctOutput.value = value;
  // Bouml preserved body end 0007AD71
}

void ClearISumIfOvershootsTarget::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007ADF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007ADF1
}

ClearISumIfPWMReachesMaximum::ClearISumIfPWMReachesMaximum() {
  // Bouml preserved body begin 0007A771
    this->name = "ClearISumIfPWMReachesMaximum";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007A771
}

ClearISumIfPWMReachesMaximum::~ClearISumIfPWMReachesMaximum() {
  // Bouml preserved body begin 0007A7F1
  // Bouml preserved body end 0007A7F1
}

void ClearISumIfPWMReachesMaximum::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0007A871
    parameter = this->value;
  // Bouml preserved body end 0007A871
}

void ClearISumIfPWMReachesMaximum::setParameter(const bool parameter) {
  // Bouml preserved body begin 0007A8F1
    this->value = parameter;
  // Bouml preserved body end 0007A8F1
}

void ClearISumIfPWMReachesMaximum::toString(std::string& value) {
  // Bouml preserved body begin 0009D171
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D171
}

void ClearISumIfPWMReachesMaximum::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007A971
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 137; //ClearISumIfPWMReachesMaximum
    message.stctOutput.value = value;
  // Bouml preserved body end 0007A971
}

void ClearISumIfPWMReachesMaximum::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007A9F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007A9F1
}

ClearMotorControllerTimeoutFlag::ClearMotorControllerTimeoutFlag() {
  // Bouml preserved body begin 0009FAF1
    this->name = "ClearMotorControllerTimeoutFlag";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
    this->value = true;
  // Bouml preserved body end 0009FAF1
}

ClearMotorControllerTimeoutFlag::~ClearMotorControllerTimeoutFlag() {
  // Bouml preserved body begin 0009FB71
  // Bouml preserved body end 0009FB71
}

bool ClearMotorControllerTimeoutFlag::getParameter() const {
  // Bouml preserved body begin 0009FBF1
  // Bouml preserved body end 0009FBF1
}

void ClearMotorControllerTimeoutFlag::setParameter() {
  // Bouml preserved body begin 0009FC71
  // Bouml preserved body end 0009FC71
}

void ClearMotorControllerTimeoutFlag::toString(std::string& value) {
  // Bouml preserved body begin 0009FCF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009FCF1
}

void ClearMotorControllerTimeoutFlag::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0009FD71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 158; //ClearMotorControllerTimeoutFlag
    message.stctOutput.value = value;
  // Bouml preserved body end 0009FD71
}

void ClearMotorControllerTimeoutFlag::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0009FDF1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value; 
    }
  // Bouml preserved body end 0009FDF1
}

ClearTargetDistance::ClearTargetDistance() {
  // Bouml preserved body begin 00079771
    this->name = "ClearTargetDistance";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00079771
}

ClearTargetDistance::~ClearTargetDistance() {
  // Bouml preserved body begin 000797F1
  // Bouml preserved body end 000797F1
}

void ClearTargetDistance::getParameter(int& parameter) const {
  // Bouml preserved body begin 00079871
    parameter = this->value;
  // Bouml preserved body end 00079871
}

void ClearTargetDistance::setParameter(const int parameter) {
  // Bouml preserved body begin 000798F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000798F1
}

void ClearTargetDistance::toString(std::string& value) {
  // Bouml preserved body begin 0009CBF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CBF1
}

void ClearTargetDistance::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00079971

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 9; //ClearTargetDistance
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 00079971
}

void ClearTargetDistance::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000799F1
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 000799F1
}

CommutationCompensationClockwise::CommutationCompensationClockwise() {
  // Bouml preserved body begin 0006F1F1
    this->name = "CommutationCompensationClockwise";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006F1F1
}

CommutationCompensationClockwise::~CommutationCompensationClockwise() {
  // Bouml preserved body begin 0006F271
  // Bouml preserved body end 0006F271
}

void CommutationCompensationClockwise::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006F2F1
    parameter = this->value;
  // Bouml preserved body end 0006F2F1
}

void CommutationCompensationClockwise::setParameter(const int parameter) {
  // Bouml preserved body begin 0006F371
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006F371
}

void CommutationCompensationClockwise::toString(std::string& value) {
  // Bouml preserved body begin 0009E2F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E2F1
}

void CommutationCompensationClockwise::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006F3F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 242; //CommutationCompensationClockwise
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 0006F3F1
}

void CommutationCompensationClockwise::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006F471
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 0006F471
}

CommutationCompensationCounterClockwise::CommutationCompensationCounterClockwise() {
  // Bouml preserved body begin 0006F5F1
    this->name = "CommutationCompensationCounterClockwise";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006F5F1
}

CommutationCompensationCounterClockwise::~CommutationCompensationCounterClockwise() {
  // Bouml preserved body begin 0006F671
  // Bouml preserved body end 0006F671
}

void CommutationCompensationCounterClockwise::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006F6F1
    parameter = this->value;
  // Bouml preserved body end 0006F6F1
}

void CommutationCompensationCounterClockwise::setParameter(const int parameter) {
  // Bouml preserved body begin 0006F771
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006F771
}

void CommutationCompensationCounterClockwise::toString(std::string& value) {
  // Bouml preserved body begin 0009E371
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E371
}

void CommutationCompensationCounterClockwise::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006F7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 243; //CommutationCompensationCounterClockwise
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 0006F7F1
}

void CommutationCompensationCounterClockwise::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006F871
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 0006F871
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
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
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
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DCF1
}

void CommutationMotorCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0008C571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 177; //CommutationMotorCurrent
    message.stctOutput.value = value.value() * 1000.0; // ampere to milli ampere

  // Bouml preserved body end 0008C571
}

void CommutationMotorCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0008C5F1
    this->value = ((double)message.stctInput.value)/1000.0 * ampere; //milli ampere to ampere
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
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CFF1
}

void CurrentControlLoopDelay::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007A171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 134; //CurrentControlLoopDelay
    message.stctOutput.value = value.value() * 1000 *1000 /50; //sec to µsec

  // Bouml preserved body end 0007A171
}

void CurrentControlLoopDelay::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007A1F1
    this->value = (((double)message.stctInput.value)/(1000 * 1000)) * 50 * seconds; //µsec to sec
  // Bouml preserved body end 0007A1F1
}

EncoderNullPolarity::EncoderNullPolarity() {
  // Bouml preserved body begin 00083F71
    this->name = "EncoderNullPolarity";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00083F71
}

EncoderNullPolarity::~EncoderNullPolarity() {
  // Bouml preserved body begin 00083FF1
  // Bouml preserved body end 00083FF1
}

void EncoderNullPolarity::getParameter(bool& parameter) const {
  // Bouml preserved body begin 00084071
    parameter = this->value;
  // Bouml preserved body end 00084071
}

void EncoderNullPolarity::setParameter(const bool parameter) {
  // Bouml preserved body begin 000840F1
    this->value = parameter;
  // Bouml preserved body end 000840F1
}

void EncoderNullPolarity::toString(std::string& value) {
  // Bouml preserved body begin 0009E6F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E6F1
}

void EncoderNullPolarity::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00084171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 252; //EncoderNullPolarity
    message.stctOutput.value = value; 

  // Bouml preserved body end 00084171
}

void EncoderNullPolarity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000841F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000841F1
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
    message.stctOutput.value = value;
  // Bouml preserved body end 00071571
}

void EncoderResolution::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072371
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
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
    message.stctOutput.value = value;

  // Bouml preserved body end 0007D671
}

void EncoderStopSwitch::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007D6F1
    this->value = message.stctInput.value;
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
    message.stctOutput.value = value;
  // Bouml preserved body end 00071E71
}

void HallSensorPolarityReversal::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000724F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
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
    message.stctOutput.value = value;
  // Bouml preserved body end 000A13F1
}

void I2tExceedCounter::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A1471
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
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
    message.stctOutput.value = value;
  // Bouml preserved body end 000A0AF1
}

void I2tLimit::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000A0B71
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 000A0B71
}

InitializationMode::InitializationMode() {
  // Bouml preserved body begin 000710F1
    this->name = "InitializationMode";
    this->lowerLimit = 0;
    this->upperLimit = 1;
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
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value; //TODO do convertion
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
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E3F1
}

void InitSineDelay::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00083171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 244; //InitSineDelay
    message.stctOutput.value = value.value() * 1000; //sec to µsec

  // Bouml preserved body end 00083171
}

void InitSineDelay::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000831F1
    this->value = (((double)message.stctInput.value)/1000)  * seconds; //µsec to sec
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
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 00082971
}

void MassInertiaConstant::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000829F1
    this->value = message.stctInput.value; //TODO do convertion
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
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CA71
}

void MaximumMotorCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006A7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 6; //MaximumMotorCurrent
    message.stctOutput.value = value.value() * 1000.0; // ampere to milli ampere

  // Bouml preserved body end 0006A7F1
}

void MaximumMotorCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006A871
    this->value = ((double)message.stctInput.value)/1000.0 * ampere; //milli ampere to ampere
  // Bouml preserved body end 0006A871
}

MaximumPWMChangePerPIDInterval::MaximumPWMChangePerPIDInterval() {
  // Bouml preserved body begin 00083771
    this->name = "MaximumPWMChangePerPIDInterval";
    this->lowerLimit = 0;
    this->upperLimit = 3599;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00083771
}

MaximumPWMChangePerPIDInterval::~MaximumPWMChangePerPIDInterval() {
  // Bouml preserved body begin 000837F1
  // Bouml preserved body end 000837F1
}

void MaximumPWMChangePerPIDInterval::getParameter(int& parameter) const {
  // Bouml preserved body begin 00083871
    parameter = this->value;
  // Bouml preserved body end 00083871
}

void MaximumPWMChangePerPIDInterval::setParameter(const int parameter) {
  // Bouml preserved body begin 000838F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000838F1
}

void MaximumPWMChangePerPIDInterval::toString(std::string& value) {
  // Bouml preserved body begin 0009E571
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E571
}

void MaximumPWMChangePerPIDInterval::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00083971

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 246; //MaximumPWMChangePerPIDInterval
    message.stctOutput.value = value;

  // Bouml preserved body end 00083971
}

void MaximumPWMChangePerPIDInterval::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000839F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000839F1
}

MaximumVelocityToSetPosition::MaximumVelocityToSetPosition() {
  // Bouml preserved body begin 00078F71
    this->name = "MaximumVelocityToSetPosition";
    this->lowerLimit = INT_MIN * radian_per_second;
    this->upperLimit = INT_MAX * radian_per_second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00078F71
}

MaximumVelocityToSetPosition::~MaximumVelocityToSetPosition() {
  // Bouml preserved body begin 00078FF1
  // Bouml preserved body end 00078FF1
}

void MaximumVelocityToSetPosition::getParameter(quantity<angular_velocity>& parameter) const {
  // Bouml preserved body begin 00079071
    parameter = this->value;
  // Bouml preserved body end 00079071
}

void MaximumVelocityToSetPosition::setParameter(const quantity<angular_velocity>& parameter) {
  // Bouml preserved body begin 000790F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000790F1
}

void MaximumVelocityToSetPosition::toString(std::string& value) {
  // Bouml preserved body begin 0009CAF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CAF1
}

void MaximumVelocityToSetPosition::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00079171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 7; //MVP Target reached velocity
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);

  // Bouml preserved body end 00079171
}

void MaximumVelocityToSetPosition::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000791F1
    double motorRPM = message.stctInput.value;
    this->value =  ((motorRPM / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 000791F1
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
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E8F1
}

void MotorCoilResistance::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00070F71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 240; //MotorCoilResistance
    message.stctOutput.value = value.value() * 1000; //from ohm to milli ohm
  // Bouml preserved body end 00070F71
}

void MotorCoilResistance::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072271
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = ((double)message.stctInput.value)/1000 * ohm;
    }
  // Bouml preserved body end 00072271
}

MotorContollerGearRatio::MotorContollerGearRatio() {
  // Bouml preserved body begin 00070AF1
    this->name = "MotorContollerGearRatio";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00070AF1
}

MotorContollerGearRatio::~MotorContollerGearRatio() {
  // Bouml preserved body begin 00070B71
  // Bouml preserved body end 00070B71
}

void MotorContollerGearRatio::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 00070BF1
    parameter = this->value;
  // Bouml preserved body end 00070BF1
}

void MotorContollerGearRatio::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 00089071
    this->value = parameter;
  // Bouml preserved body end 00089071
}

void MotorContollerGearRatio::toString(std::string& value) {
  // Bouml preserved body begin 0009E471
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E471
}

void MotorContollerGearRatio::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00070C71
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 211; //MotorContollerGearRatio
    message.stctOutput.value = value;
  // Bouml preserved body end 00070C71
}

void MotorContollerGearRatio::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000721F1
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 000721F1
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
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009F871
}

void MotorControllerTimeout::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0009F8F1


    if (msgType = SAP)
      message.stctOutput.commandNumber = SGP;
    else if (msgType = GAP) {
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
    message.stctOutput.value = value;
  // Bouml preserved body end 00071B71
}

void MotorPoles::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072471
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
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
  ss << this->name << ": " << this->value;
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

PIDControllerState::PIDControllerState() {
  // Bouml preserved body begin 000700F1
    this->name = "ArePIDcontrollersActive";
    this->lowerLimit = 0;
    this->upperLimit = 2;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000700F1
}

PIDControllerState::~PIDControllerState() {
  // Bouml preserved body begin 00070171
  // Bouml preserved body end 00070171
}

void PIDControllerState::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000701F1
    parameter = this->value;
  // Bouml preserved body end 000701F1
}

void PIDControllerState::setParameter(const bool parameter) {
  // Bouml preserved body begin 000933F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000933F1
}

void PIDControllerState::toString(std::string& value) {
  // Bouml preserved body begin 0009E771
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E771
}

void PIDControllerState::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 000702F1
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 147; //ArePIDcontrollersActive
    message.stctOutput.value = value;
  // Bouml preserved body end 000702F1
}

void PIDControllerState::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00070371
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value;
    }
  // Bouml preserved body end 00070371
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
  ss << this->name << ": " << this->value;
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

PositionTargetReachedDistance::PositionTargetReachedDistance() {
  // Bouml preserved body begin 00079B71
    this->name = "PositionTargetReachedDistance";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00079B71
}

PositionTargetReachedDistance::~PositionTargetReachedDistance() {
  // Bouml preserved body begin 00079BF1
  // Bouml preserved body end 00079BF1
}

void PositionTargetReachedDistance::getParameter(int& parameter) const {
  // Bouml preserved body begin 00079C71
    parameter = this->value;
  // Bouml preserved body end 00079C71
}

void PositionTargetReachedDistance::setParameter(const int parameter) {
  // Bouml preserved body begin 00079CF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00079CF1
}

void PositionTargetReachedDistance::toString(std::string& value) {
  // Bouml preserved body begin 0009CC71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CC71
}

void PositionTargetReachedDistance::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00079D71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 10; //PositionTargetReachedDistance
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 00079D71
}

void PositionTargetReachedDistance::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00079DF1
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 00079DF1
}

PWMHysteresis::PWMHysteresis() {
  // Bouml preserved body begin 0007A371
    this->name = "PWMHysteresis";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007A371
}

PWMHysteresis::~PWMHysteresis() {
  // Bouml preserved body begin 0007A3F1
  // Bouml preserved body end 0007A3F1
}

void PWMHysteresis::getParameter(int& parameter) const {
  // Bouml preserved body begin 0007A471
    parameter = this->value;
  // Bouml preserved body end 0007A471
}

void PWMHysteresis::setParameter(const int parameter) {
  // Bouml preserved body begin 0007A4F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007A4F1
}

void PWMHysteresis::toString(std::string& value) {
  // Bouml preserved body begin 0009D0F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D0F1
}

void PWMHysteresis::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007A571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 136; //PWMHysteresis
    message.stctOutput.value = value; 

  // Bouml preserved body end 0007A571
}

void PWMHysteresis::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007A5F1
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0007A5F1
}

PWMLimit::PWMLimit() {
  // Bouml preserved body begin 00079371
    this->name = "PWMLimit";
    this->lowerLimit = 0;
    this->upperLimit = 100;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00079371
}

PWMLimit::~PWMLimit() {
  // Bouml preserved body begin 000793F1
  // Bouml preserved body end 000793F1
}

void PWMLimit::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 00079471
    parameter = this->value;
  // Bouml preserved body end 00079471
}

void PWMLimit::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 000794F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000794F1
}

void PWMLimit::toString(std::string& value) {
  // Bouml preserved body begin 0009C9F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C9F1
}

void PWMLimit::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00079571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 5; //PWMLimit
    message.stctOutput.value = (((double)value)/100.0)*3599;

  // Bouml preserved body end 00079571
}

void PWMLimit::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000795F1
    this->value = ((double)message.stctInput.value/3599.0)*100;
  // Bouml preserved body end 000795F1
}

PWMSchemeBlockCommutation::PWMSchemeBlockCommutation() {
  // Bouml preserved body begin 000707F1
    this->name = "PWMSchemeBlockCommutation";
    this->lowerLimit = 0;
    this->upperLimit = 2;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000707F1
}

PWMSchemeBlockCommutation::~PWMSchemeBlockCommutation() {
  // Bouml preserved body begin 00070871
  // Bouml preserved body end 00070871
}

void PWMSchemeBlockCommutation::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000708F1
    parameter = this->value;
  // Bouml preserved body end 000708F1
}

void PWMSchemeBlockCommutation::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 000934F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }
    this->value = parameter;
  // Bouml preserved body end 000934F1
}

void PWMSchemeBlockCommutation::toString(std::string& value) {
  // Bouml preserved body begin 0009E871
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E871
}

void PWMSchemeBlockCommutation::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00070971
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 167; //PWMSchemeBlockCommutation
    message.stctOutput.value = value;
  // Bouml preserved body end 00070971
}

void PWMSchemeBlockCommutation::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00072171
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
      this->value = message.stctInput.value; //TODO do convertion
    }
  // Bouml preserved body end 00072171
}

ReinitializationSinusoidalCommutation::ReinitializationSinusoidalCommutation() {
  // Bouml preserved body begin 0006C9F1
    this->name = "ReinitializationSinusoidalCommutation";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006C9F1
}

ReinitializationSinusoidalCommutation::~ReinitializationSinusoidalCommutation() {
  // Bouml preserved body begin 0006CA71
  // Bouml preserved body end 0006CA71
}

void ReinitializationSinusoidalCommutation::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0006CAF1
    parameter = this->value;
  // Bouml preserved body end 0006CAF1
}

void ReinitializationSinusoidalCommutation::setParameter(const bool parameter) {
  // Bouml preserved body begin 0006CB71
    this->value = parameter;
  // Bouml preserved body end 0006CB71
}

void ReinitializationSinusoidalCommutation::toString(std::string& value) {
  // Bouml preserved body begin 0009D4F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D4F1
}

void ReinitializationSinusoidalCommutation::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006CBF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 160; //ReinitializationSinusoidalCommutation
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 0006CBF1
}

void ReinitializationSinusoidalCommutation::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006CC71
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 0006CC71
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
    if (message.stctOutput.commandNumber == message.stctInput.commandNumber && message.stctInput.status == NO_ERROR) {
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
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 0007CA71
}

void SetEncoderCounterZeroAtNextNChannel::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007CAF1
    this->value = message.stctInput.value; //TODO do convertion
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
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 0007CE71
}

void SetEncoderCounterZeroAtNextSwitch::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007CEF1
    this->value = message.stctInput.value; //TODO do convertion
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
    message.stctOutput.typeNumber = 163; //ReinitializationSinusoidalCommutation
    message.stctOutput.value = value; //TODO do convertion

  // Bouml preserved body end 0007D271
}

void SetEncoderCounterZeroOnlyOnce::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007D2F1
    this->value = message.stctInput.value; //TODO do convertion
  // Bouml preserved body end 0007D2F1
}

SineCompensationFactor::SineCompensationFactor() {
  // Bouml preserved body begin 00083B71
    this->name = "SineCompensationFactor";
    this->lowerLimit = 0;
    this->upperLimit = 255;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00083B71
}

SineCompensationFactor::~SineCompensationFactor() {
  // Bouml preserved body begin 00083BF1
  // Bouml preserved body end 00083BF1
}

void SineCompensationFactor::getParameter(int& parameter) const {
  // Bouml preserved body begin 00083C71
    parameter = this->value;
  // Bouml preserved body end 00083C71
}

void SineCompensationFactor::setParameter(const int parameter) {
  // Bouml preserved body begin 00083CF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00083CF1
}

void SineCompensationFactor::toString(std::string& value) {
  // Bouml preserved body begin 0009E5F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E5F1
}

void SineCompensationFactor::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00083D71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 247; //SineCompensationFactor
    message.stctOutput.value = value;

  // Bouml preserved body end 00083D71
}

void SineCompensationFactor::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00083DF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 00083DF1
}

SineInitializationVelocity::SineInitializationVelocity() {
  // Bouml preserved body begin 0006EDF1
    this->name = "SineInitializationVelocity";
    this->lowerLimit = INT_MIN * radian_per_second;
    this->upperLimit = INT_MAX * radian_per_second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006EDF1
}

SineInitializationVelocity::~SineInitializationVelocity() {
  // Bouml preserved body begin 0006EE71
  // Bouml preserved body end 0006EE71
}

void SineInitializationVelocity::getParameter(quantity<angular_velocity>& parameter) const {
  // Bouml preserved body begin 0006EEF1
    parameter = this->value;
  // Bouml preserved body end 0006EEF1
}

void SineInitializationVelocity::setParameter(const quantity<angular_velocity>& parameter) {
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
    message.stctOutput.typeNumber = 241; //BlockCommutationMaximumSpeed
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);
  // Bouml preserved body end 0006EFF1
}

void SineInitializationVelocity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006F071
    double motorRPM = message.stctInput.value;
    this->value =  ((motorRPM / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
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
  ss << this->name << ": " << this->value;
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


} // namespace youbot
