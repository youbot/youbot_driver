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
#include "youbot/YouBotJointParameter.hpp"
namespace youbot {

YouBotJointParameter::YouBotJointParameter() {
  // Bouml preserved body begin 0005BB71
  // Bouml preserved body end 0005BB71
}

YouBotJointParameter::~YouBotJointParameter() {
  // Bouml preserved body begin 0005BBF1
  // Bouml preserved body end 0005BBF1
}

void YouBotJointParameter::toString(std::string& value) {
  // Bouml preserved body begin 0009C3F1
  // Bouml preserved body end 0009C3F1
}

JointName::JointName() {
  // Bouml preserved body begin 0005C0F1
    this->name = "JointName";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 0005C0F1
}

JointName::~JointName() {
  // Bouml preserved body begin 0005C171
  // Bouml preserved body end 0005C171
}

void JointName::getParameter(std::string& parameter) const {
  // Bouml preserved body begin 0005C1F1
    parameter = this->value;
  // Bouml preserved body end 0005C1F1
}

void JointName::setParameter(const std::string parameter) {
  // Bouml preserved body begin 0005C271
    this->value = parameter;
  // Bouml preserved body end 0005C271
}

void JointName::toString(std::string& value) {
  // Bouml preserved body begin 0009C471
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C471
}

InitializeJoint::InitializeJoint() {
  // Bouml preserved body begin 00095171
    this->name = "InitializeJoint";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00095171
}

InitializeJoint::~InitializeJoint() {
  // Bouml preserved body begin 000951F1
  // Bouml preserved body end 000951F1
}

void InitializeJoint::getParameter(bool& parameter) const {
  // Bouml preserved body begin 00095271
    parameter = this->value;
  // Bouml preserved body end 00095271
}

void InitializeJoint::setParameter(const bool parameter) {
  // Bouml preserved body begin 000952F1
    this->value = parameter;
  // Bouml preserved body end 000952F1
}

void InitializeJoint::toString(std::string& value) {
  // Bouml preserved body begin 0009C971
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C971
}

void InitializeJoint::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00095371
    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 15; //Initialize BLDC
    message.stctOutput.value = (int)this->value;
  // Bouml preserved body end 00095371
}

void InitializeJoint::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000953F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000953F1
}

CalibrateJoint::CalibrateJoint() {
  // Bouml preserved body begin 00061F71
    this->name = "CalibrateJoint";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 00061F71
}

CalibrateJoint::~CalibrateJoint() {
  // Bouml preserved body begin 00061FF1
  // Bouml preserved body end 00061FF1
}

void CalibrateJoint::getParameter(bool& doCalibration, CalibrationDirection& calibrationDirection, quantity<si::current>& maxCurrent) const {
  // Bouml preserved body begin 00062071
    doCalibration = this->doCalibration;
    calibrationDirection = this->calibrationDirection;
    maxCurrent = this->maxCurrent;
  // Bouml preserved body end 00062071
}

void CalibrateJoint::setParameter(const bool doCalibration, CalibrationDirection calibrationDirection, const quantity<si::current>& maxCurrent) {
  // Bouml preserved body begin 000620F1
    this->doCalibration = doCalibration;
    this->calibrationDirection = calibrationDirection;
    this->maxCurrent = maxCurrent;
  // Bouml preserved body end 000620F1
}

void CalibrateJoint::toString(std::string& value) {
  // Bouml preserved body begin 0009C771
  std::stringstream ss;
  ss << this->name << ": " << "doCalibration " <<this->doCalibration << " calibrationDirection "<< this->calibrationDirection << " maxCurrent " << this->maxCurrent;
  value  = ss.str();
  // Bouml preserved body end 0009C771
}

FirmwareVersion::FirmwareVersion() {
  // Bouml preserved body begin 00098D71
    this->name = "FirmwareVersion";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00098D71
}

FirmwareVersion::~FirmwareVersion() {
  // Bouml preserved body begin 00098DF1
  // Bouml preserved body end 00098DF1
}

void FirmwareVersion::getParameter(int& controllerType, double& firmwareVersion) const {
  // Bouml preserved body begin 00098E71
    controllerType = this->controllerType;
    firmwareVersion = this->firmwareVersion;
  // Bouml preserved body end 00098E71
}

void FirmwareVersion::setParameter(const int controllerType, const double firmwareVersion) {
  // Bouml preserved body begin 00098EF1
    this->controllerType = controllerType;
    this->firmwareVersion = firmwareVersion;
  // Bouml preserved body end 00098EF1
}

void FirmwareVersion::toString(std::string& value) {
  // Bouml preserved body begin 0009C571
  std::stringstream ss;
  ss << this->name << ": " << this->controllerType << " Version: " << this->firmwareVersion;
  value  = ss.str();
  // Bouml preserved body end 0009C571
}

void FirmwareVersion::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00098F71
    message.stctOutput.commandNumber = FIRMWARE_VERSION;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 0;
    message.stctOutput.value = 0;
  // Bouml preserved body end 00098F71
}

void FirmwareVersion::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00098FF1
  // Bouml preserved body end 00098FF1
}

GearRatio::GearRatio() {
  // Bouml preserved body begin 0005BDF1
    this->name = "GearRatio";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 0005BDF1
}

GearRatio::~GearRatio() {
  // Bouml preserved body begin 0005BE71
  // Bouml preserved body end 0005BE71
}

void GearRatio::getParameter(double& parameter) const {
  // Bouml preserved body begin 0005BEF1
    parameter = this->value;
  // Bouml preserved body end 0005BEF1
}

void GearRatio::setParameter(const double parameter) {
  // Bouml preserved body begin 0005BF71
    if (parameter == 0) {
      throw std::out_of_range("A Gear Ratio of 0 is not allowed");
    }
    this->value = parameter;
  // Bouml preserved body end 0005BF71
}

void GearRatio::toString(std::string& value) {
  // Bouml preserved body begin 0009C5F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C5F1
}

EncoderTicksPerRound::EncoderTicksPerRound() {
  // Bouml preserved body begin 0005C3F1
    this->name = "EncoderTicksPerRound";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 0005C3F1
}

EncoderTicksPerRound::~EncoderTicksPerRound() {
  // Bouml preserved body begin 0005C471
  // Bouml preserved body end 0005C471
}

void EncoderTicksPerRound::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0005C4F1
    parameter = this->value;
  // Bouml preserved body end 0005C4F1
}

void EncoderTicksPerRound::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 0005C571
    if (parameter == 0) {
      throw std::out_of_range("Zero Encoder Ticks per Round are not allowed");
    }
    this->value = parameter;
  // Bouml preserved body end 0005C571
}

void EncoderTicksPerRound::toString(std::string& value) {
  // Bouml preserved body begin 0009C671
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C671
}

InverseMovementDirection::InverseMovementDirection() {
  // Bouml preserved body begin 0005C6F1
    this->name = "InverseMovementDirection";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 0005C6F1
}

InverseMovementDirection::~InverseMovementDirection() {
  // Bouml preserved body begin 0005C771
  // Bouml preserved body end 0005C771
}

void InverseMovementDirection::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0005C7F1
    parameter = this->value;
  // Bouml preserved body end 0005C7F1
}

void InverseMovementDirection::setParameter(const bool parameter) {
  // Bouml preserved body begin 0005C871
    this->value = parameter;
  // Bouml preserved body end 0005C871
}

void InverseMovementDirection::toString(std::string& value) {
  // Bouml preserved body begin 0009C6F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C6F1
}

JointLimits::JointLimits() {
  // Bouml preserved body begin 00063EF1
    this->name = "JointLimits";
    this->parameterType = API_PARAMETER;
    this->lowerLimit = 0;
    this->upperLimit = 0;
    this->areLimitsActive = true;
  // Bouml preserved body end 00063EF1
}

JointLimits::~JointLimits() {
  // Bouml preserved body begin 00063F71
  // Bouml preserved body end 00063F71
}

void JointLimits::getParameter(int& lowerLimit, int& upperLimit, bool& areLimitsActive) const {
  // Bouml preserved body begin 00063FF1
    lowerLimit = this->lowerLimit;
    upperLimit = this->upperLimit;
    areLimitsActive = this->areLimitsActive;
  // Bouml preserved body end 00063FF1
}

void JointLimits::setParameter(const int lowerLimit, const int upperLimit, const bool activateLimits) {
  // Bouml preserved body begin 00064071
    if (lowerLimit > upperLimit) {
      throw std::out_of_range("The lower joint limit it not allowed to be bigger than the upper limit");
    }
    this->lowerLimit = lowerLimit;
    this->upperLimit = upperLimit;
    this->areLimitsActive = activateLimits;
  // Bouml preserved body end 00064071
}

void JointLimits::toString(std::string& value) {
  // Bouml preserved body begin 0009C7F1
  std::stringstream ss;
  ss << this->name << ": lower Limit: " << this->lowerLimit  << " upper Limit: " << this->upperLimit;
  value  = ss.str();
  // Bouml preserved body end 0009C7F1
}

TorqueConstant::TorqueConstant() {
  // Bouml preserved body begin 000C71F1
    this->name = "TorqueConstant";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 000C71F1
}

TorqueConstant::~TorqueConstant() {
  // Bouml preserved body begin 000C7271
  // Bouml preserved body end 000C7271
}

void TorqueConstant::getParameter(double& parameter) const {
  // Bouml preserved body begin 000C72F1
    parameter = this->value;
  // Bouml preserved body end 000C72F1
}

void TorqueConstant::setParameter(const double parameter) {
  // Bouml preserved body begin 000C7371
    if (parameter == 0 || parameter < 0) {
      throw std::out_of_range("It is not allowed to set a zero or negative torque constant");
    }
    this->value = parameter;
  // Bouml preserved body end 000C7371
}

void TorqueConstant::toString(std::string& value) {
  // Bouml preserved body begin 000C73F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000C73F1
}

MaximumPositioningVelocity::MaximumPositioningVelocity() {
  // Bouml preserved body begin 0005A171
    this->name = "MaximumPositioningVelocity";
    this->lowerLimit = INT_MIN * radian_per_second;
    this->upperLimit = INT_MAX * radian_per_second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0005A171
}

MaximumPositioningVelocity::~MaximumPositioningVelocity() {
  // Bouml preserved body begin 0005A1F1
  // Bouml preserved body end 0005A1F1
}

void MaximumPositioningVelocity::getParameter(quantity<angular_velocity>& parameter) const {
  // Bouml preserved body begin 00059CF1
    parameter = this->value;
  // Bouml preserved body end 00059CF1
}

void MaximumPositioningVelocity::setParameter(const quantity<angular_velocity>& parameter) {
  // Bouml preserved body begin 00059C71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00059C71
}

void MaximumPositioningVelocity::toString(std::string& value) {
  // Bouml preserved body begin 0009C4F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009C4F1
}

void MaximumPositioningVelocity::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0005A0F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 4; //maximum positioning speed
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);

  // Bouml preserved body end 0005A0F1
}

void MaximumPositioningVelocity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0005A071
    double motorRPM = message.stctInput.value;
    this->value =  ((motorRPM / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 0005A071
}

MotorAcceleration::MotorAcceleration() {
  // Bouml preserved body begin 0006A9F1
    this->name = "MotorAcceleration";
    this->lowerLimit = INT_MIN * radian_per_second/second;
    this->upperLimit = INT_MAX * radian_per_second/second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006A9F1
}

MotorAcceleration::~MotorAcceleration() {
  // Bouml preserved body begin 0006AA71
  // Bouml preserved body end 0006AA71
}

void MotorAcceleration::getParameter(quantity<angular_acceleration>& parameter) const {
  // Bouml preserved body begin 0006AAF1
    parameter = this->value;
  // Bouml preserved body end 0006AAF1
}

void MotorAcceleration::setParameter(const quantity<angular_acceleration>& parameter) {
  // Bouml preserved body begin 0006AB71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006AB71
}

void MotorAcceleration::toString(std::string& value) {
  // Bouml preserved body begin 0009CCF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CCF1
}

void MotorAcceleration::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006ABF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 11; //MotorAcceleration
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);

  // Bouml preserved body end 0006ABF1
}

void MotorAcceleration::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006AC71
    double motorRPMperSec = message.stctInput.value;
    this->value =  ((motorRPMperSec / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second/second;
  // Bouml preserved body end 0006AC71
}

RampGeneratorSpeedAndPositionControl::RampGeneratorSpeedAndPositionControl() {
  // Bouml preserved body begin 0006C5F1
    this->name = "RampGeneratorSpeedAndPositionControl";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006C5F1
}

RampGeneratorSpeedAndPositionControl::~RampGeneratorSpeedAndPositionControl() {
  // Bouml preserved body begin 0006C671
  // Bouml preserved body end 0006C671
}

void RampGeneratorSpeedAndPositionControl::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0006C6F1
    parameter = this->value;
  // Bouml preserved body end 0006C6F1
}

void RampGeneratorSpeedAndPositionControl::setParameter(const bool parameter) {
  // Bouml preserved body begin 0006C771
    this->value = parameter;
  // Bouml preserved body end 0006C771
}

void RampGeneratorSpeedAndPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D471
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D471
}

void RampGeneratorSpeedAndPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006C7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 146; //RampGeneratorSpeedAndPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006C7F1
}

void RampGeneratorSpeedAndPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006C871
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006C871
}

PositionControlSwitchingThreshold::PositionControlSwitchingThreshold() {
  // Bouml preserved body begin 0006F9F1
    this->name = "PositionControlSwitchingThreshold";
    this->lowerLimit = INT_MIN * radian_per_second;
    this->upperLimit = INT_MAX * radian_per_second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006F9F1
}

PositionControlSwitchingThreshold::~PositionControlSwitchingThreshold() {
  // Bouml preserved body begin 0006FA71
  // Bouml preserved body end 0006FA71
}

void PositionControlSwitchingThreshold::getParameter(quantity<angular_velocity>& parameter) const {
  // Bouml preserved body begin 0006FAF1
    parameter = this->value;
  // Bouml preserved body end 0006FAF1
}

void PositionControlSwitchingThreshold::setParameter(const quantity<angular_velocity>& parameter) {
  // Bouml preserved body begin 0006FB71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006FB71
}

void PositionControlSwitchingThreshold::toString(std::string& value) {
  // Bouml preserved body begin 0009CD71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CD71
}

void PositionControlSwitchingThreshold::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006FBF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 12; //PositionControlSwitchingThreshold
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);

  // Bouml preserved body end 0006FBF1
}

void PositionControlSwitchingThreshold::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006FC71
    double motorRPM = message.stctInput.value;
    this->value =  ((motorRPM / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 0006FC71
}

SpeedControlSwitchingThreshold::SpeedControlSwitchingThreshold() {
  // Bouml preserved body begin 0006A1F1
    this->name = "SpeedControlSwitchingThreshold";
    this->lowerLimit = INT_MIN * radian_per_second;
    this->upperLimit = INT_MAX * radian_per_second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006A1F1
}

SpeedControlSwitchingThreshold::~SpeedControlSwitchingThreshold() {
  // Bouml preserved body begin 0006A271
  // Bouml preserved body end 0006A271
}

void SpeedControlSwitchingThreshold::getParameter(quantity<angular_velocity>& parameter) const {
  // Bouml preserved body begin 0006A2F1
    parameter = this->value;
  // Bouml preserved body end 0006A2F1
}

void SpeedControlSwitchingThreshold::setParameter(const quantity<angular_velocity>& parameter) {
  // Bouml preserved body begin 0006A371
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006A371
}

void SpeedControlSwitchingThreshold::toString(std::string& value) {
  // Bouml preserved body begin 0009CB71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CB71
}

void SpeedControlSwitchingThreshold::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006A3F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 8; //SpeedControlSwitchingThreshold
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);

  // Bouml preserved body end 0006A3F1
}

void SpeedControlSwitchingThreshold::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006A471
    double motorRPM = message.stctInput.value;
    this->value =  ((motorRPM / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 0006A471
}

CurrentControlSwitchingThreshold::CurrentControlSwitchingThreshold() {
  // Bouml preserved body begin 00081371
    this->name = "CurrentControlSwitchingThreshold";
    this->lowerLimit = INT_MIN * radian_per_second;
    this->upperLimit = INT_MAX * radian_per_second;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00081371
}

CurrentControlSwitchingThreshold::~CurrentControlSwitchingThreshold() {
  // Bouml preserved body begin 000813F1
  // Bouml preserved body end 000813F1
}

void CurrentControlSwitchingThreshold::getParameter(quantity<angular_velocity>& parameter) const {
  // Bouml preserved body begin 00081471
    parameter = this->value;
  // Bouml preserved body end 00081471
}

void CurrentControlSwitchingThreshold::setParameter(const quantity<angular_velocity>& parameter) {
  // Bouml preserved body begin 000814F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000814F1
}

void CurrentControlSwitchingThreshold::toString(std::string& value) {
  // Bouml preserved body begin 0009DC71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DC71
}

void CurrentControlSwitchingThreshold::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00081571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 176; //CurrentControlSwitchingThreshold
    message.stctOutput.value = (int32) round((value.value() / (storage.gearRatio * 2.0 * M_PI)) * 60.0);

  // Bouml preserved body end 00081571
}

void CurrentControlSwitchingThreshold::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000815F1
    double motorRPM = message.stctInput.value;
    this->value =  ((motorRPM / 60.0) * storage.gearRatio * 2.0 * M_PI) * radian_per_second;
  // Bouml preserved body end 000815F1
}

PParameterFirstParametersPositionControl::PParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 0005C9F1
    this->name = "PParameterFirstParametersPositionControl";
    this->lowerLimit = 0;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0005C9F1
}

PParameterFirstParametersPositionControl::~PParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 0005CA71
  // Bouml preserved body end 0005CA71
}

void PParameterFirstParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0005CAF1
    parameter = this->value;
  // Bouml preserved body end 0005CAF1
}

void PParameterFirstParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0005CB71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0005CB71
}

void PParameterFirstParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009CDF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CDF1
}

void PParameterFirstParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0005CBF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 130; //PParameterFirstParametersPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0005CBF1
}

void PParameterFirstParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0005CC71
    this->value = message.stctInput.value;
  // Bouml preserved body end 0005CC71
}

IParameterFirstParametersPositionControl::IParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 000699F1
    this->name = "IParameterFirstParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000699F1
}

IParameterFirstParametersPositionControl::~IParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 00069A71
  // Bouml preserved body end 00069A71
}

void IParameterFirstParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00069AF1
    parameter = this->value;
  // Bouml preserved body end 00069AF1
}

void IParameterFirstParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 00069B71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00069B71
}

void IParameterFirstParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009CE71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CE71
}

void IParameterFirstParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00069BF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 131; //IParameterFirstParametersPositionControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 00069BF1
}

void IParameterFirstParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00069C71
    this->value = message.stctInput.value;
  // Bouml preserved body end 00069C71
}

DParameterFirstParametersPositionControl::DParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 00069DF1
    this->name = "DParameterFirstParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00069DF1
}

DParameterFirstParametersPositionControl::~DParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 00069E71
  // Bouml preserved body end 00069E71
}

void DParameterFirstParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00069EF1
    parameter = this->value;
  // Bouml preserved body end 00069EF1
}

void DParameterFirstParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 00069F71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00069F71
}

void DParameterFirstParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009CEF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009CEF1
}

void DParameterFirstParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00069FF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 132; //PParameterFirstParametersPositionControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 00069FF1
}

void DParameterFirstParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006A071
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0006A071
}

IClippingParameterFirstParametersPositionControl::IClippingParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 0006B1F1
    this->name = "IClippingParameterFirstParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006B1F1
}

IClippingParameterFirstParametersPositionControl::~IClippingParameterFirstParametersPositionControl() {
  // Bouml preserved body begin 0006B271
  // Bouml preserved body end 0006B271
}

void IClippingParameterFirstParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006B2F1
    parameter = this->value;
  // Bouml preserved body end 0006B2F1
}

void IClippingParameterFirstParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006B371
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006B371
}

void IClippingParameterFirstParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D071
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D071
}

void IClippingParameterFirstParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006B3F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 135; //IClippingParameterFirstParametersPositionControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 0006B3F1
}

void IClippingParameterFirstParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006B471
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0006B471
}

PParameterFirstParametersSpeedControl::PParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006B5F1
    this->name = "PParameterFirstParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006B5F1
}

PParameterFirstParametersSpeedControl::~PParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006B671
  // Bouml preserved body end 0006B671
}

void PParameterFirstParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006B6F1
    parameter = this->value;
  // Bouml preserved body end 0006B6F1
}

void PParameterFirstParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006B771
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006B771
}

void PParameterFirstParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D271
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D271
}

void PParameterFirstParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006B7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 140; //PParameterFirstParametersSpeedControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 0006B7F1
}

void PParameterFirstParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006B871
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0006B871
}

IParameterFirstParametersSpeedControl::IParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006B9F1
    this->name = "IParameterFirstParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006B9F1
}

IParameterFirstParametersSpeedControl::~IParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006BA71
  // Bouml preserved body end 0006BA71
}

void IParameterFirstParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006BAF1
    parameter = this->value;
  // Bouml preserved body end 0006BAF1
}

void IParameterFirstParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006BB71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006BB71
}

void IParameterFirstParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D2F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D2F1
}

void IParameterFirstParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006BBF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 141; //IParameterFirstParametersSpeedControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 0006BBF1
}

void IParameterFirstParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006BC71
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0006BC71
}

DParameterFirstParametersSpeedControl::DParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006BDF1
    this->name = "DParameterFirstParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006BDF1
}

DParameterFirstParametersSpeedControl::~DParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006BE71
  // Bouml preserved body end 0006BE71
}

void DParameterFirstParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006BEF1
    parameter = this->value;
  // Bouml preserved body end 0006BEF1
}

void DParameterFirstParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006BF71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006BF71
}

void DParameterFirstParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D371
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D371
}

void DParameterFirstParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006BFF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 142; //DParameterFirstParametersSpeedControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 0006BFF1
}

void DParameterFirstParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006C071
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0006C071
}

IClippingParameterFirstParametersSpeedControl::IClippingParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006C1F1
    this->name = "IClippingParameterFirstParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006C1F1
}

IClippingParameterFirstParametersSpeedControl::~IClippingParameterFirstParametersSpeedControl() {
  // Bouml preserved body begin 0006C271
  // Bouml preserved body end 0006C271
}

void IClippingParameterFirstParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006C2F1
    parameter = this->value;
  // Bouml preserved body end 0006C2F1
}

void IClippingParameterFirstParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006C371
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006C371
}

void IClippingParameterFirstParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D3F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D3F1
}

void IClippingParameterFirstParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006C3F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 143; //IClippingParameterFirstParametersSpeedControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 0006C3F1
}

void IClippingParameterFirstParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006C471
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0006C471
}

PParameterFirstParametersCurrentControl::PParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007F371
    this->name = "PParameterFirstParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007F371
}

PParameterFirstParametersCurrentControl::~PParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007F3F1
  // Bouml preserved body end 0007F3F1
}

void PParameterFirstParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0007F471
    parameter = this->value;
  // Bouml preserved body end 0007F471
}

void PParameterFirstParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0007F4F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007F4F1
}

void PParameterFirstParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D871
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D871
}

void PParameterFirstParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007F571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 168; //PParameterFirstParametersCurrentControl
    message.stctOutput.value = value; 

  // Bouml preserved body end 0007F571
}

void PParameterFirstParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007F5F1
    this->value = message.stctInput.value; 
  // Bouml preserved body end 0007F5F1
}

IParameterFirstParametersCurrentControl::IParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007F771
    this->name = "IParameterFirstParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007F771
}

IParameterFirstParametersCurrentControl::~IParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007F7F1
  // Bouml preserved body end 0007F7F1
}

void IParameterFirstParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0007F871
    parameter = this->value;
  // Bouml preserved body end 0007F871
}

void IParameterFirstParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0007F8F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007F8F1
}

void IParameterFirstParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D8F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D8F1
}

void IParameterFirstParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007F971

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 169; //IParameterFirstParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0007F971
}

void IParameterFirstParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007F9F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007F9F1
}

DParameterFirstParametersCurrentControl::DParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007FB71
    this->name = "DParameterFirstParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007FB71
}

DParameterFirstParametersCurrentControl::~DParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007FBF1
  // Bouml preserved body end 0007FBF1
}

void DParameterFirstParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0007FC71
    parameter = this->value;
  // Bouml preserved body end 0007FC71
}

void DParameterFirstParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0007FCF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0007FCF1
}

void DParameterFirstParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D971
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D971
}

void DParameterFirstParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0007FD71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 170; //DParameterFirstParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0007FD71
}

void DParameterFirstParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0007FDF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 0007FDF1
}

IClippingParameterFirstParametersCurrentControl::IClippingParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007FF71
    this->name = "IClippingParameterFirstParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0007FF71
}

IClippingParameterFirstParametersCurrentControl::~IClippingParameterFirstParametersCurrentControl() {
  // Bouml preserved body begin 0007FFF1
  // Bouml preserved body end 0007FFF1
}

void IClippingParameterFirstParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00080071
    parameter = this->value;
  // Bouml preserved body end 00080071
}

void IClippingParameterFirstParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 000800F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000800F1
}

void IClippingParameterFirstParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009D9F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009D9F1
}

void IClippingParameterFirstParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00080171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 171; //IClippingParameterFirstParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 00080171
}

void IClippingParameterFirstParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000801F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000801F1
}

PParameterSecondParametersPositionControl::PParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006CDF1
    this->name = "PParameterSecondParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006CDF1
}

PParameterSecondParametersPositionControl::~PParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006CE71
  // Bouml preserved body end 0006CE71
}

void PParameterSecondParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006CEF1
    parameter = this->value;
  // Bouml preserved body end 0006CEF1
}

void PParameterSecondParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006CF71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006CF71
}

void PParameterSecondParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DD71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DD71
}

void PParameterSecondParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006CFF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 230; //PParameterFirstParametersPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006CFF1
}

void PParameterSecondParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006D071
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006D071
}

IParameterSecondParametersPositionControl::IParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006D1F1
    this->name = "IParameterSecondParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006D1F1
}

IParameterSecondParametersPositionControl::~IParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006D271
  // Bouml preserved body end 0006D271
}

void IParameterSecondParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006D2F1
    parameter = this->value;
  // Bouml preserved body end 0006D2F1
}

void IParameterSecondParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006D371
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006D371
}

void IParameterSecondParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DDF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DDF1
}

void IParameterSecondParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006D3F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 231; //IParameterSecondParametersPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006D3F1
}

void IParameterSecondParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006D471
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006D471
}

DParameterSecondParametersPositionControl::DParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006D5F1
    this->name = "DParameterSecondParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006D5F1
}

DParameterSecondParametersPositionControl::~DParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006D671
  // Bouml preserved body end 0006D671
}

void DParameterSecondParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006D6F1
    parameter = this->value;
  // Bouml preserved body end 0006D6F1
}

void DParameterSecondParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006D771
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006D771
}

void DParameterSecondParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DE71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DE71
}

void DParameterSecondParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006D7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 232; //DParameterSecondParametersPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006D7F1
}

void DParameterSecondParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006D871
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006D871
}

IClippingParameterSecondParametersPositionControl::IClippingParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006D9F1
    this->name = "IClippingParameterSecondParametersPositionControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006D9F1
}

IClippingParameterSecondParametersPositionControl::~IClippingParameterSecondParametersPositionControl() {
  // Bouml preserved body begin 0006DA71
  // Bouml preserved body end 0006DA71
}

void IClippingParameterSecondParametersPositionControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006DAF1
    parameter = this->value;
  // Bouml preserved body end 0006DAF1
}

void IClippingParameterSecondParametersPositionControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006DB71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006DB71
}

void IClippingParameterSecondParametersPositionControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DEF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DEF1
}

void IClippingParameterSecondParametersPositionControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006DBF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 233; //IClippingParameterSecondParametersPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006DBF1
}

void IClippingParameterSecondParametersPositionControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006DC71
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006DC71
}

PParameterSecondParametersSpeedControl::PParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006DDF1
    this->name = "PParameterSecondParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006DDF1
}

PParameterSecondParametersSpeedControl::~PParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006DE71
  // Bouml preserved body end 0006DE71
}

void PParameterSecondParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006DEF1
    parameter = this->value;
  // Bouml preserved body end 0006DEF1
}

void PParameterSecondParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006DF71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006DF71
}

void PParameterSecondParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DF71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DF71
}

void PParameterSecondParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006DFF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 234; //PParameterSecondParametersSpeedControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006DFF1
}

void PParameterSecondParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006E071
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006E071
}

IParameterSecondParametersSpeedControl::IParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006E1F1
    this->name = "IParameterSecondParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006E1F1
}

IParameterSecondParametersSpeedControl::~IParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006E271
  // Bouml preserved body end 0006E271
}

void IParameterSecondParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006E2F1
    parameter = this->value;
  // Bouml preserved body end 0006E2F1
}

void IParameterSecondParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006E371
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006E371
}

void IParameterSecondParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DFF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DFF1
}

void IParameterSecondParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006E3F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 235; //IParameterFirstParametersPositionControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006E3F1
}

void IParameterSecondParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006E471
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006E471
}

DParameterSecondParametersSpeedControl::DParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006E5F1
    this->name = "DParameterSecondParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006E5F1
}

DParameterSecondParametersSpeedControl::~DParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006E671
  // Bouml preserved body end 0006E671
}

void DParameterSecondParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006E6F1
    parameter = this->value;
  // Bouml preserved body end 0006E6F1
}

void DParameterSecondParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006E771
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006E771
}

void DParameterSecondParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009E071
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E071
}

void DParameterSecondParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006E7F1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 236; //DParameterSecondParametersSpeedControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006E7F1
}

void DParameterSecondParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006E871
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006E871
}

IClippingParameterSecondParametersSpeedControl::IClippingParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006E9F1
    this->name = "IClippingParameterSecondParametersSpeedControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0006E9F1
}

IClippingParameterSecondParametersSpeedControl::~IClippingParameterSecondParametersSpeedControl() {
  // Bouml preserved body begin 0006EA71
  // Bouml preserved body end 0006EA71
}

void IClippingParameterSecondParametersSpeedControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 0006EAF1
    parameter = this->value;
  // Bouml preserved body end 0006EAF1
}

void IClippingParameterSecondParametersSpeedControl::setParameter(const int parameter) {
  // Bouml preserved body begin 0006EB71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 0006EB71
}

void IClippingParameterSecondParametersSpeedControl::toString(std::string& value) {
  // Bouml preserved body begin 0009E0F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009E0F1
}

void IClippingParameterSecondParametersSpeedControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 0006EBF1

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 237; //IClippingParameterSecondParametersSpeedControl
    message.stctOutput.value = value;

  // Bouml preserved body end 0006EBF1
}

void IClippingParameterSecondParametersSpeedControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 0006EC71
    this->value = message.stctInput.value;
  // Bouml preserved body end 0006EC71
}

PParameterSecondParametersCurrentControl::PParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 00080371
    this->name = "PParameterSecondParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00080371
}

PParameterSecondParametersCurrentControl::~PParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 000803F1
  // Bouml preserved body end 000803F1
}

void PParameterSecondParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00080471
    parameter = this->value;
  // Bouml preserved body end 00080471
}

void PParameterSecondParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 000804F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000804F1
}

void PParameterSecondParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DA71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DA71
}

void PParameterSecondParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00080571

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 172; //PParameterSecondParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 00080571
}

void PParameterSecondParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000805F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000805F1
}

IParameterSecondParametersCurrentControl::IParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 00080771
    this->name = "IParameterSecondParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00080771
}

IParameterSecondParametersCurrentControl::~IParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 000807F1
  // Bouml preserved body end 000807F1
}

void IParameterSecondParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00080871
    parameter = this->value;
  // Bouml preserved body end 00080871
}

void IParameterSecondParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 000808F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000808F1
}

void IParameterSecondParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DAF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DAF1
}

void IParameterSecondParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00080971

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 173; //IParameterSecondParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 00080971
}

void IParameterSecondParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000809F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000809F1
}

DParameterSecondParametersCurrentControl::DParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 00080B71
    this->name = "DParameterSecondParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00080B71
}

DParameterSecondParametersCurrentControl::~DParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 00080BF1
  // Bouml preserved body end 00080BF1
}

void DParameterSecondParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00080C71
    parameter = this->value;
  // Bouml preserved body end 00080C71
}

void DParameterSecondParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 00080CF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00080CF1
}

void DParameterSecondParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DB71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DB71
}

void DParameterSecondParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00080D71

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 174; //DParameterSecondParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 00080D71
}

void DParameterSecondParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 00080DF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 00080DF1
}

IClippingParameterSecondParametersCurrentControl::IClippingParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 00080F71
    this->name = "IClippingParameterSecondParametersCurrentControl";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00080F71
}

IClippingParameterSecondParametersCurrentControl::~IClippingParameterSecondParametersCurrentControl() {
  // Bouml preserved body begin 00080FF1
  // Bouml preserved body end 00080FF1
}

void IClippingParameterSecondParametersCurrentControl::getParameter(int& parameter) const {
  // Bouml preserved body begin 00081071
    parameter = this->value;
  // Bouml preserved body end 00081071
}

void IClippingParameterSecondParametersCurrentControl::setParameter(const int parameter) {
  // Bouml preserved body begin 000810F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000810F1
}

void IClippingParameterSecondParametersCurrentControl::toString(std::string& value) {
  // Bouml preserved body begin 0009DBF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009DBF1
}

void IClippingParameterSecondParametersCurrentControl::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message, TMCLCommandNumber msgType, const YouBotJointStorage& storage) const {
  // Bouml preserved body begin 00081171

    message.stctOutput.commandNumber = msgType;
    message.stctOutput.moduleAddress = DRIVE;
    message.stctOutput.typeNumber = 175; //IClippingParameterSecondParametersCurrentControl
    message.stctOutput.value = value;

  // Bouml preserved body end 00081171
}

void IClippingParameterSecondParametersCurrentControl::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message, const YouBotJointStorage& storage) {
  // Bouml preserved body begin 000811F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000811F1
}


} // namespace youbot
