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

StopJoint::StopJoint() {
  // Bouml preserved body begin 00065C71
    this->name = "StopJoint";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 00065C71
}

StopJoint::~StopJoint() {
  // Bouml preserved body begin 00065CF1
  // Bouml preserved body end 00065CF1
}

void StopJoint::getParameter(bool& parameter) const {
  // Bouml preserved body begin 00065D71
    parameter = this->value;
  // Bouml preserved body end 00065D71
}

void StopJoint::setParameter(const bool parameter) {
  // Bouml preserved body begin 00065DF1
    this->value = parameter;
  // Bouml preserved body end 00065DF1
}

NoMoreAction::NoMoreAction() {
  // Bouml preserved body begin 00066071
    this->name = "NoMoreAction";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 00066071
}

NoMoreAction::~NoMoreAction() {
  // Bouml preserved body begin 000660F1
  // Bouml preserved body end 000660F1
}

void NoMoreAction::getParameter(bool& parameter) const {
  // Bouml preserved body begin 00066171
    parameter = this->value;
  // Bouml preserved body end 00066171
}

void NoMoreAction::setParameter(const bool parameter) {
  // Bouml preserved body begin 000661F1
    this->value = parameter;
  // Bouml preserved body end 000661F1
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

void DParameterFirstParametersSpeedControl::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 0006BEF1
    parameter = this->value;
  // Bouml preserved body end 0006BEF1
}

void DParameterFirstParametersSpeedControl::setParameter(const unsigned int parameter) {
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

RampGeneratorSpeedAndPositionControl::RampGeneratorSpeedAndPositionControl() {
  // Bouml preserved body begin 0006C5F1
    this->name = "ActivateRampGeneratorSpeedAndPositionControl";
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


} // namespace youbot
