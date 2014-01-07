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
#include "youbot_driver/youbot/YouBotGripperParameter.hpp"
namespace youbot {

YouBotGripperParameter::YouBotGripperParameter() {
  // Bouml preserved body begin 0005F0F1
  // Bouml preserved body end 0005F0F1
}

YouBotGripperParameter::~YouBotGripperParameter() {
  // Bouml preserved body begin 0005F171
  // Bouml preserved body end 0005F171
}

GripperFirmwareVersion::GripperFirmwareVersion() {
  // Bouml preserved body begin 000BEAF1
    this->name = "FirmwareVersion";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BEAF1
}

GripperFirmwareVersion::~GripperFirmwareVersion() {
  // Bouml preserved body begin 000BEB71
  // Bouml preserved body end 000BEB71
}

void GripperFirmwareVersion::getParameter(int& controllerType, double& firmwareVersion) const {
  // Bouml preserved body begin 000BEBF1
    controllerType = this->controllerType;
    firmwareVersion = this->firmwareVersion;
  // Bouml preserved body end 000BEBF1
}

void GripperFirmwareVersion::setParameter(const int controllerType, const double firmwareVersion) {
  // Bouml preserved body begin 000BEC71
    this->controllerType = controllerType;
    this->firmwareVersion = firmwareVersion;
  // Bouml preserved body end 000BEC71
}

void GripperFirmwareVersion::toString(std::string& value) const {
  // Bouml preserved body begin 000BECF1
  std::stringstream ss;
  ss << this->name << ": " << this->controllerType << " Version: " << this->firmwareVersion;
  value  = ss.str();
  // Bouml preserved body end 000BECF1
}

void GripperFirmwareVersion::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BED71
    message.stctOutput.commandNumber = FIRMWARE_VERSION;
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.typeNumber = 0;  //GripperFirmwareVersion
    message.stctOutput.motorNumber = 0;
    message.stctOutput.value = 0;
  // Bouml preserved body end 000BED71
}

void GripperFirmwareVersion::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BEDF1
  // Bouml preserved body end 000BEDF1
}

GripperBarName::GripperBarName() {
  // Bouml preserved body begin 00109D71
    this->name = "GripperBarName";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 00109D71
}

GripperBarName::~GripperBarName() {
  // Bouml preserved body begin 00109DF1
  // Bouml preserved body end 00109DF1
}

void GripperBarName::getParameter(std::string& parameter) const {
  // Bouml preserved body begin 00109E71
    parameter = this->value;
  // Bouml preserved body end 00109E71
}

void GripperBarName::setParameter(const std::string parameter) {
  // Bouml preserved body begin 00109EF1
    this->value = parameter;
  // Bouml preserved body end 00109EF1
}

void GripperBarName::toString(std::string& value) const {
  // Bouml preserved body begin 00109F71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 00109F71
}

void GripperBarName::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 00109FF1
  // Bouml preserved body end 00109FF1
}

void GripperBarName::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 0010A071
  // Bouml preserved body end 0010A071
}

CalibrateGripper::CalibrateGripper() {
  // Bouml preserved body begin 0005F3F1
    this->name = "CalibrateGripper";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 0005F3F1
}

CalibrateGripper::~CalibrateGripper() {
  // Bouml preserved body begin 0005F471
  // Bouml preserved body end 0005F471
}

void CalibrateGripper::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0005F4F1
    parameter = this->value;
  // Bouml preserved body end 0005F4F1
}

void CalibrateGripper::setParameter(const bool parameter) {
  // Bouml preserved body begin 0005F571
    this->value = parameter;
  // Bouml preserved body end 0005F571
}

void CalibrateGripper::toString(std::string& value) const {
  // Bouml preserved body begin 0009F171
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009F171
}

void CalibrateGripper::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BE6F1
  // Bouml preserved body end 000BE6F1
}

void CalibrateGripper::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BE8F1
  // Bouml preserved body end 000BE8F1
}

BarSpacingOffset::BarSpacingOffset() {
  // Bouml preserved body begin 0005FC71
    this->name = "BarSpacingOffset";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 0005FC71
}

BarSpacingOffset::~BarSpacingOffset() {
  // Bouml preserved body begin 0005FCF1
  // Bouml preserved body end 0005FCF1
}

void BarSpacingOffset::getParameter(quantity<si::length>& parameter) const {
  // Bouml preserved body begin 0005FD71
    parameter = this->value;
  // Bouml preserved body end 0005FD71
}

void BarSpacingOffset::setParameter(const quantity<si::length>& parameter) {
  // Bouml preserved body begin 0005FDF1
  if(parameter > 1 *meter || parameter < 0 *meter){
    throw std::out_of_range("The Bar Spacing Offset is only allowed to be less than 1m and bigger than zero");
  }
    this->value = parameter;
  // Bouml preserved body end 0005FDF1
}

void BarSpacingOffset::toString(std::string& value) const {
  // Bouml preserved body begin 0009F1F1
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009F1F1
}

void BarSpacingOffset::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BE871
  // Bouml preserved body end 000BE871
}

void BarSpacingOffset::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BEA71
  // Bouml preserved body end 000BEA71
}

MaxEncoderValue::MaxEncoderValue() {
  // Bouml preserved body begin 00061B71
    this->name = "MaxEncoderValue";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 00061B71
}

MaxEncoderValue::~MaxEncoderValue() {
  // Bouml preserved body begin 00061BF1
  // Bouml preserved body end 00061BF1
}

void MaxEncoderValue::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 00061C71
    parameter = this->value;
  // Bouml preserved body end 00061C71
}

void MaxEncoderValue::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 00061CF1
    this->value = parameter;
  // Bouml preserved body end 00061CF1
}

void MaxEncoderValue::toString(std::string& value) const {
  // Bouml preserved body begin 0009F0F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0009F0F1
}

void MaxEncoderValue::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BE7F1
  // Bouml preserved body end 000BE7F1
}

void MaxEncoderValue::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BE9F1
  // Bouml preserved body end 000BE9F1
}

MaxTravelDistance::MaxTravelDistance() {
  // Bouml preserved body begin 000618F1
    this->name = "MaxTravelDistance";
    this->parameterType = API_PARAMETER;
  // Bouml preserved body end 000618F1
}

MaxTravelDistance::~MaxTravelDistance() {
  // Bouml preserved body begin 00061971
  // Bouml preserved body end 00061971
}

void MaxTravelDistance::getParameter(quantity<si::length>& parameter) const {
  // Bouml preserved body begin 000619F1
    parameter = this->value;
  // Bouml preserved body end 000619F1
}

void MaxTravelDistance::setParameter(const quantity<si::length>& parameter) {
  // Bouml preserved body begin 00061A71
  if(parameter > 1 *meter || parameter < 0 *meter){
    throw std::out_of_range("The Max Travel Distance is only allowed to be less than 1m and bigger than zero");
  }
    this->value = parameter;
  // Bouml preserved body end 00061A71
}

void MaxTravelDistance::toString(std::string& value) const {
  // Bouml preserved body begin 0009F071
  std::stringstream ss;
  ss << this->name << ": " << this->value.value();
  value  = ss.str();
  // Bouml preserved body end 0009F071
}

void MaxTravelDistance::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BE771
  // Bouml preserved body end 000BE771
}

void MaxTravelDistance::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BE971
  // Bouml preserved body end 000BE971
}

ActualPosition::ActualPosition() {
  // Bouml preserved body begin 000E10F1
    this->name = "ActualPosition";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000E10F1
}

ActualPosition::~ActualPosition() {
  // Bouml preserved body begin 000E1171
  // Bouml preserved body end 000E1171
}

void ActualPosition::getParameter(int& parameter) const {
  // Bouml preserved body begin 000E11F1
    parameter = this->value;
  // Bouml preserved body end 000E11F1
}

void ActualPosition::setParameter(const int parameter) {
  // Bouml preserved body begin 000E1271
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000E1271
}

void ActualPosition::toString(std::string& value) const {
  // Bouml preserved body begin 000E12F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000E12F1
}

void ActualPosition::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000E1371
    message.stctOutput.typeNumber = 1; //ActualPosition
    message.stctOutput.value = (uint32)(value * -1);

  // Bouml preserved body end 000E1371
}

void ActualPosition::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000E13F1
    this->value = (int32)message.stctInput.value;
    this->value = this->value * -1;
  // Bouml preserved body end 000E13F1
}

PositionSetpoint::PositionSetpoint() {
  // Bouml preserved body begin 000E19F1
    this->name = "PositionSetpoint";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000E19F1
}

PositionSetpoint::~PositionSetpoint() {
  // Bouml preserved body begin 000E1A71
  // Bouml preserved body end 000E1A71
}

void PositionSetpoint::getParameter(int& parameter) const {
  // Bouml preserved body begin 000E1AF1
    parameter = this->value;
  // Bouml preserved body end 000E1AF1
}

void PositionSetpoint::setParameter(const int parameter) {
  // Bouml preserved body begin 000E1B71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000E1B71
}

void PositionSetpoint::toString(std::string& value) const {
  // Bouml preserved body begin 000E1BF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000E1BF1
}

void PositionSetpoint::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000E1C71

    message.stctOutput.typeNumber = 0; //PositionSetpoint
    message.stctOutput.value = (uint32)(value * -1);

  // Bouml preserved body end 000E1C71
}

void PositionSetpoint::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000E1CF1
    this->value = (int)message.stctInput.value;
    this->value = this->value * -1;

  // Bouml preserved body end 000E1CF1
}

TargetPositionReached::TargetPositionReached() {
  // Bouml preserved body begin 000FFE71
    this->name = "TargetPositionReached";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000FFE71
}

TargetPositionReached::~TargetPositionReached() {
  // Bouml preserved body begin 000FFEF1
  // Bouml preserved body end 000FFEF1
}

void TargetPositionReached::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000FFF71
    parameter = this->value;
  // Bouml preserved body end 000FFF71
}

void TargetPositionReached::toString(std::string& value) const {
  // Bouml preserved body begin 00100071
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 00100071
}

void TargetPositionReached::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 001000F1
    message.stctOutput.typeNumber = 8;  //TargetPositionReached
    if(value)
      message.stctOutput.value = 1;
    else
      message.stctOutput.value = 0;
  // Bouml preserved body end 001000F1
}

void TargetPositionReached::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 00100171
    this->value = message.stctInput.value;
  // Bouml preserved body end 00100171
}

ActualVelocity::ActualVelocity() {
  // Bouml preserved body begin 000E1571
    this->name = "ActualVelocity";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000E1571
}

ActualVelocity::~ActualVelocity() {
  // Bouml preserved body begin 000E15F1
  // Bouml preserved body end 000E15F1
}

void ActualVelocity::getParameter(int& parameter) const {
  // Bouml preserved body begin 000E1671
    parameter = this->value;
  // Bouml preserved body end 000E1671
}

void ActualVelocity::setParameter(const int parameter) {
  // Bouml preserved body begin 000E16F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000E16F1
}

void ActualVelocity::toString(std::string& value) const {
  // Bouml preserved body begin 000E1771
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000E1771
}

void ActualVelocity::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000E17F1

    message.stctOutput.typeNumber = 3;  //ActualVelocity
    message.stctOutput.value = (uint32)value;

  // Bouml preserved body end 000E17F1
}

void ActualVelocity::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000E1871
    this->value = (int32)message.stctInput.value;

  // Bouml preserved body end 000E1871
}

VelocitySetpoint::VelocitySetpoint() {
  // Bouml preserved body begin 000E1E71
    this->name = "VelocitySetpoint";
    this->lowerLimit = INT_MIN;
    this->upperLimit = INT_MAX;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000E1E71
}

VelocitySetpoint::~VelocitySetpoint() {
  // Bouml preserved body begin 000E1EF1
  // Bouml preserved body end 000E1EF1
}

void VelocitySetpoint::getParameter(int& parameter) const {
  // Bouml preserved body begin 000E1F71
    parameter = this->value;
  // Bouml preserved body end 000E1F71
}

void VelocitySetpoint::setParameter(const int parameter) {
  // Bouml preserved body begin 000E1FF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000E1FF1
}

void VelocitySetpoint::toString(std::string& value) const {
  // Bouml preserved body begin 000E2071
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000E2071
}

void VelocitySetpoint::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000E20F1
    message.stctOutput.typeNumber = 2;  //VelocitySetpoint
    message.stctOutput.value = (uint32)value;

  // Bouml preserved body end 000E20F1
}

void VelocitySetpoint::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000E2171
    this->value = (int32)message.stctInput.value;

  // Bouml preserved body end 000E2171
}

ActualLoadValue::ActualLoadValue() {
  // Bouml preserved body begin 000BBDF1
    this->name = "ActualLoadValue";
    this->lowerLimit = 0;
    this->upperLimit = 1023;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BBDF1
}

ActualLoadValue::~ActualLoadValue() {
  // Bouml preserved body begin 000BBE71
  // Bouml preserved body end 000BBE71
}

void ActualLoadValue::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BBEF1
    parameter = this->value;
  // Bouml preserved body end 000BBEF1
}

void ActualLoadValue::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BBF71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BBF71
}

void ActualLoadValue::toString(std::string& value) const {
  // Bouml preserved body begin 000BBFF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BBFF1
}

void ActualLoadValue::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BC071

    message.stctOutput.typeNumber = 206;  //ActualLoadValue
    message.stctOutput.value = value;

  // Bouml preserved body end 000BC071
}

void ActualLoadValue::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BC0F1
    this->value  = message.stctInput.value; 
  // Bouml preserved body end 000BC0F1
}

ChopperBlankTime::ChopperBlankTime() {
  // Bouml preserved body begin 000B5AF1
    this->name = "ChopperBlankTime";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B5AF1
}

ChopperBlankTime::~ChopperBlankTime() {
  // Bouml preserved body begin 000B5B71
  // Bouml preserved body end 000B5B71
}

void ChopperBlankTime::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B5BF1
    parameter = this->value;
  // Bouml preserved body end 000B5BF1
}

void ChopperBlankTime::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B5C71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B5C71
}

void ChopperBlankTime::toString(std::string& value) const {
  // Bouml preserved body begin 000B5CF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B5CF1
}

void ChopperBlankTime::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B5D71

    message.stctOutput.typeNumber = 162;  //ChopperBlankTime
    message.stctOutput.value = value;

  // Bouml preserved body end 000B5D71
}

void ChopperBlankTime::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B5DF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B5DF1
}

ChopperHysteresisDecrement::ChopperHysteresisDecrement() {
  // Bouml preserved body begin 000B63F1
    this->name = "ChopperHysteresisDecrement";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B63F1
}

ChopperHysteresisDecrement::~ChopperHysteresisDecrement() {
  // Bouml preserved body begin 000B6471
  // Bouml preserved body end 000B6471
}

void ChopperHysteresisDecrement::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B64F1
    parameter = this->value;
  // Bouml preserved body end 000B64F1
}

void ChopperHysteresisDecrement::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B6571
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B6571
}

void ChopperHysteresisDecrement::toString(std::string& value) const {
  // Bouml preserved body begin 000B65F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B65F1
}

void ChopperHysteresisDecrement::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B6671
    message.stctOutput.typeNumber = 164;  //ChopperHysteresisDecrement
    message.stctOutput.value = value;
  // Bouml preserved body end 000B6671
}

void ChopperHysteresisDecrement::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B66F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B66F1
}

ChopperHysteresisEnd::ChopperHysteresisEnd() {
  // Bouml preserved body begin 000B6871
    this->name = "ChopperHysteresisEnd";
    this->lowerLimit = -3;
    this->upperLimit = 12;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B6871
}

ChopperHysteresisEnd::~ChopperHysteresisEnd() {
  // Bouml preserved body begin 000B68F1
  // Bouml preserved body end 000B68F1
}

void ChopperHysteresisEnd::getParameter(int& parameter) const {
  // Bouml preserved body begin 000B6971
    parameter = this->value;
  // Bouml preserved body end 000B6971
}

void ChopperHysteresisEnd::setParameter(const int parameter) {
  // Bouml preserved body begin 000B69F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B69F1
}

void ChopperHysteresisEnd::toString(std::string& value) const {
  // Bouml preserved body begin 000B6A71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B6A71
}

void ChopperHysteresisEnd::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B6AF1
    message.stctOutput.typeNumber = 165;  //ChopperHysteresisEnd
    message.stctOutput.value = value;
  // Bouml preserved body end 000B6AF1
}

void ChopperHysteresisEnd::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B6B71
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 000B6B71
}

ChopperHysteresisStart::ChopperHysteresisStart() {
  // Bouml preserved body begin 00107271
    this->name = "ChopperHysteresisStart";
    this->lowerLimit = 0;
    this->upperLimit = 8;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00107271
}

ChopperHysteresisStart::~ChopperHysteresisStart() {
  // Bouml preserved body begin 001072F1
  // Bouml preserved body end 001072F1
}

void ChopperHysteresisStart::getParameter(int& parameter) const {
  // Bouml preserved body begin 00107371
    parameter = this->value;
  // Bouml preserved body end 00107371
}

void ChopperHysteresisStart::setParameter(const int parameter) {
  // Bouml preserved body begin 001073F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 001073F1
}

void ChopperHysteresisStart::toString(std::string& value) const {
  // Bouml preserved body begin 00107471
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 00107471
}

void ChopperHysteresisStart::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 001074F1
    message.stctOutput.typeNumber = 166;  //ChopperHysteresisStart
    message.stctOutput.value = value;
  // Bouml preserved body end 001074F1
}

void ChopperHysteresisStart::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 00107571
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 00107571
}

ChopperMode::ChopperMode() {
  // Bouml preserved body begin 000B5F71
    this->name = "ChopperMode";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B5F71
}

ChopperMode::~ChopperMode() {
  // Bouml preserved body begin 000B5FF1
  // Bouml preserved body end 000B5FF1
}

void ChopperMode::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000B6071
    parameter = this->value;
  // Bouml preserved body end 000B6071
}

void ChopperMode::setParameter(const bool parameter) {
  // Bouml preserved body begin 000B60F1

    this->value = parameter;
  // Bouml preserved body end 000B60F1
}

void ChopperMode::toString(std::string& value) const {
  // Bouml preserved body begin 000B6171
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B6171
}

void ChopperMode::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B61F1
    message.stctOutput.typeNumber = 163;  //ChopperMode
    message.stctOutput.value = value;
  // Bouml preserved body end 000B61F1
}

void ChopperMode::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B6271
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B6271
}

ChopperOffTime::ChopperOffTime() {
  // Bouml preserved body begin 000B6CF1
    this->name = "ChopperOffTime";
    this->lowerLimit = 0;
    this->upperLimit = 15;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B6CF1
}

ChopperOffTime::~ChopperOffTime() {
  // Bouml preserved body begin 000B6D71
  // Bouml preserved body end 000B6D71
}

void ChopperOffTime::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B6DF1
    parameter = this->value;
  // Bouml preserved body end 000B6DF1
}

void ChopperOffTime::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B6E71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (parameter == 1) {
      throw std::out_of_range("One is not allowed for this parameter");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B6E71
}

void ChopperOffTime::toString(std::string& value) const {
  // Bouml preserved body begin 000B6EF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B6EF1
}

void ChopperOffTime::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B6F71
    message.stctOutput.typeNumber = 167;   //ChopperOffTime
    message.stctOutput.value = value;
  // Bouml preserved body end 000B6F71
}

void ChopperOffTime::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B6FF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B6FF1
}

DoubleStepEnable::DoubleStepEnable() {
  // Bouml preserved body begin 000B5671
    this->name = "DoubleStepEnable";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B5671
}

DoubleStepEnable::~DoubleStepEnable() {
  // Bouml preserved body begin 000B56F1
  // Bouml preserved body end 000B56F1
}

void DoubleStepEnable::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000B5771
    parameter = this->value;
  // Bouml preserved body end 000B5771
}

void DoubleStepEnable::setParameter(const bool parameter) {
  // Bouml preserved body begin 000B57F1
    this->value = parameter;
  // Bouml preserved body end 000B57F1
}

void DoubleStepEnable::toString(std::string& value) const {
  // Bouml preserved body begin 000B5871
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B5871
}

void DoubleStepEnable::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B58F1
    message.stctOutput.typeNumber = 161;  //DoubleStepEnable
    message.stctOutput.value = value;
  // Bouml preserved body end 000B58F1
}

void DoubleStepEnable::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B5971
    this->value = message.stctInput.value;

  // Bouml preserved body end 000B5971
}

ErrorFlags::ErrorFlags() {
  // Bouml preserved body begin 000BC271
    this->name = "ErrorFlags";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BC271
}

ErrorFlags::~ErrorFlags() {
  // Bouml preserved body begin 000BC2F1
  // Bouml preserved body end 000BC2F1
}

void ErrorFlags::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BC371
    parameter = this->value;
  // Bouml preserved body end 000BC371
}

void ErrorFlags::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BC3F1

    this->value = parameter;
  // Bouml preserved body end 000BC3F1
}

void ErrorFlags::toString(std::string& value) const {
  // Bouml preserved body begin 000BC471
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BC471
}

void ErrorFlags::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BC4F1
    message.stctOutput.typeNumber = 208;   //ErrorFlags
    message.stctOutput.value = value;
  // Bouml preserved body end 000BC4F1
}

void ErrorFlags::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BC571
    this->value = message.stctInput.value;

  // Bouml preserved body end 000BC571
}

Freewheeling::Freewheeling() {
  // Bouml preserved body begin 000BB971
    this->name = "Freewheeling";
    this->lowerLimit = 0;
    this->upperLimit = 65535;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BB971
}

Freewheeling::~Freewheeling() {
  // Bouml preserved body begin 000BB9F1
  // Bouml preserved body end 000BB9F1
}

void Freewheeling::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BBA71
    parameter = this->value;
  // Bouml preserved body end 000BBA71
}

void Freewheeling::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BBAF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BBAF1
}

void Freewheeling::toString(std::string& value) const {
  // Bouml preserved body begin 000BBB71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BBB71
}

void Freewheeling::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BBBF1
    message.stctOutput.typeNumber = 204;   //Freewheeling
    message.stctOutput.value = value;
  // Bouml preserved body end 000BBBF1
}

void Freewheeling::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BBC71
    this->value = message.stctInput.value;

  // Bouml preserved body end 000BBC71
}

MaximumAcceleration::MaximumAcceleration() {
  // Bouml preserved body begin 000B3271
    this->name = "MaximumAcceleration";
    this->lowerLimit = 0;
    this->upperLimit = 2047;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B3271
}

MaximumAcceleration::~MaximumAcceleration() {
  // Bouml preserved body begin 000B32F1
  // Bouml preserved body end 000B32F1
}

void MaximumAcceleration::getParameter(int& parameter) const {
  // Bouml preserved body begin 000B3371
    parameter = this->value;
  // Bouml preserved body end 000B3371
}

void MaximumAcceleration::setParameter(const int parameter) {
  // Bouml preserved body begin 000B33F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B33F1
}

void MaximumAcceleration::toString(std::string& value) const {
  // Bouml preserved body begin 000B3471
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B3471
}

void MaximumAcceleration::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B34F1
    message.stctOutput.typeNumber = 5;  //MaximumAcceleration
    message.stctOutput.value = value;
  // Bouml preserved body end 000B34F1
}

void MaximumAcceleration::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B3571
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B3571
}

MaximumCurrent::MaximumCurrent() {
  // Bouml preserved body begin 000B36F1
    this->name = "MaximumCurrent";
    this->lowerLimit = 0;
    this->upperLimit = 255;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B36F1
}

MaximumCurrent::~MaximumCurrent() {
  // Bouml preserved body begin 000B3771
  // Bouml preserved body end 000B3771
}

void MaximumCurrent::getParameter(int& parameter) const {
  // Bouml preserved body begin 000B37F1
    parameter = this->value;
  // Bouml preserved body end 000B37F1
}

void MaximumCurrent::setParameter(const int parameter) {
  // Bouml preserved body begin 000B3871
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B3871
}

void MaximumCurrent::toString(std::string& value) const {
  // Bouml preserved body begin 000B38F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B38F1
}

void MaximumCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B3971
    message.stctOutput.typeNumber = 6;  //MaximumCurrent
    message.stctOutput.value = value;
  // Bouml preserved body end 000B3971
}

void MaximumCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B39F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B39F1
}

MaximumPositioningSpeed::MaximumPositioningSpeed() {
  // Bouml preserved body begin 000B2DF1
    this->name = "MaximumPositioningSpeed";
    this->lowerLimit = 0;
    this->upperLimit = 2047;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B2DF1
}

MaximumPositioningSpeed::~MaximumPositioningSpeed() {
  // Bouml preserved body begin 000B2E71
  // Bouml preserved body end 000B2E71
}

void MaximumPositioningSpeed::getParameter(int& parameter) const {
  // Bouml preserved body begin 000B2EF1
    parameter = this->value;
  // Bouml preserved body end 000B2EF1
}

void MaximumPositioningSpeed::setParameter(const int parameter) {
  // Bouml preserved body begin 000B2F71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B2F71
}

void MaximumPositioningSpeed::toString(std::string& value) const {
  // Bouml preserved body begin 000B2FF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B2FF1
}

void MaximumPositioningSpeed::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B3071
    message.stctOutput.typeNumber = 4; //MaximumPositioningSpeed
    message.stctOutput.value = value;
  // Bouml preserved body end 000B3071
}

void MaximumPositioningSpeed::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B30F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B30F1
}

MicrostepResolution::MicrostepResolution() {
  // Bouml preserved body begin 000B4471
    this->name = "MicrostepResolution";
    this->lowerLimit = 0;
    this->upperLimit = 8;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B4471
}

MicrostepResolution::~MicrostepResolution() {
  // Bouml preserved body begin 000B44F1
  // Bouml preserved body end 000B44F1
}

void MicrostepResolution::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B4571
    parameter = this->value;
  // Bouml preserved body end 000B4571
}

void MicrostepResolution::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B45F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B45F1
}

void MicrostepResolution::toString(std::string& value) const {
  // Bouml preserved body begin 000B4671
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B4671
}

void MicrostepResolution::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B46F1
    message.stctOutput.typeNumber = 140;  //MicrostepResolution
    message.stctOutput.value = value;
  // Bouml preserved body end 000B46F1
}

void MicrostepResolution::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B4771
    this->value = message.stctInput.value;

  // Bouml preserved body end 000B4771
}

PowerDownDelay::PowerDownDelay() {
  // Bouml preserved body begin 000BC6F1
    this->name = "PowerDownDelay";
    this->lowerLimit = 1;
    this->upperLimit = 65535;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BC6F1
}

PowerDownDelay::~PowerDownDelay() {
  // Bouml preserved body begin 000BC771
  // Bouml preserved body end 000BC771
}

void PowerDownDelay::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BC7F1
    parameter = this->value;
  // Bouml preserved body end 000BC7F1
}

void PowerDownDelay::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BC871
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BC871
}

void PowerDownDelay::toString(std::string& value) const {
  // Bouml preserved body begin 000BC8F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BC8F1
}

void PowerDownDelay::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BC971
    message.stctOutput.typeNumber = 214;  //PowerDownDelay
    message.stctOutput.value = value;
  // Bouml preserved body end 000BC971
}

void PowerDownDelay::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BC9F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BC9F1
}

PulseDivisor::PulseDivisor() {
  // Bouml preserved body begin 000B4D71
    this->name = "PulseDivisor";
    this->lowerLimit = 0;
    this->upperLimit = 13;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B4D71
}

PulseDivisor::~PulseDivisor() {
  // Bouml preserved body begin 000B4DF1
  // Bouml preserved body end 000B4DF1
}

void PulseDivisor::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B4E71
    parameter = this->value;
  // Bouml preserved body end 000B4E71
}

void PulseDivisor::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B4EF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B4EF1
}

void PulseDivisor::toString(std::string& value) const {
  // Bouml preserved body begin 000B4F71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B4F71
}

void PulseDivisor::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B4FF1
    message.stctOutput.typeNumber = 154;  //PulseDivisor
    message.stctOutput.value = value;
  // Bouml preserved body end 000B4FF1
}

void PulseDivisor::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B5071
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B5071
}

RampDivisor::RampDivisor() {
  // Bouml preserved body begin 000B48F1
    this->name = "RampDivisor";
    this->lowerLimit = 0;
    this->upperLimit = 13;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B48F1
}

RampDivisor::~RampDivisor() {
  // Bouml preserved body begin 000B4971
  // Bouml preserved body end 000B4971
}

void RampDivisor::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B49F1
    parameter = this->value;
  // Bouml preserved body end 000B49F1
}

void RampDivisor::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B4A71
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B4A71
}

void RampDivisor::toString(std::string& value) const {
  // Bouml preserved body begin 000B4AF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B4AF1
}

void RampDivisor::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B4B71
    message.stctOutput.typeNumber = 153;  //RampDivisor
    message.stctOutput.value = value;
  // Bouml preserved body end 000B4B71
}

void RampDivisor::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B4BF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B4BF1
}

RampMode::RampMode() {
  // Bouml preserved body begin 000B3FF1
    this->name = "RampMode";
    this->lowerLimit = 0;
    this->upperLimit = 2;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B3FF1
}

RampMode::~RampMode() {
  // Bouml preserved body begin 000B4071
  // Bouml preserved body end 000B4071
}

void RampMode::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B40F1
    parameter = this->value;
  // Bouml preserved body end 000B40F1
}

void RampMode::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B4171
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B4171
}

void RampMode::toString(std::string& value) const {
  // Bouml preserved body begin 000B41F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B41F1
}

void RampMode::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B4271
    message.stctOutput.typeNumber = 138;  //RampMode
    message.stctOutput.value = value;
  // Bouml preserved body end 000B4271
}

void RampMode::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B42F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B42F1
}

ShortDetectionTimer::ShortDetectionTimer() {
  // Bouml preserved body begin 000B9E71
    this->name = "ShortDetectionTimer";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B9E71
}

ShortDetectionTimer::~ShortDetectionTimer() {
  // Bouml preserved body begin 000B9EF1
  // Bouml preserved body end 000B9EF1
}

void ShortDetectionTimer::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B9F71
    parameter = this->value;
  // Bouml preserved body end 000B9F71
}

void ShortDetectionTimer::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B9FF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B9FF1
}

void ShortDetectionTimer::toString(std::string& value) const {
  // Bouml preserved body begin 000BA071
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BA071
}

void ShortDetectionTimer::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BA0F1
    message.stctOutput.typeNumber = 178;  //ShortDetectionTimer
    message.stctOutput.value = value;
  // Bouml preserved body end 000BA0F1
}

void ShortDetectionTimer::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BA171
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BA171
}

ShortProtectionDisable::ShortProtectionDisable() {
  // Bouml preserved body begin 000B99F1
    this->name = "ShortProtectionDisable";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B99F1
}

ShortProtectionDisable::~ShortProtectionDisable() {
  // Bouml preserved body begin 000B9A71
  // Bouml preserved body end 000B9A71
}

void ShortProtectionDisable::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000B9AF1
    parameter = this->value;
  // Bouml preserved body end 000B9AF1
}

void ShortProtectionDisable::setParameter(const bool parameter) {
  // Bouml preserved body begin 000B9B71
    this->value = parameter;
  // Bouml preserved body end 000B9B71
}

void ShortProtectionDisable::toString(std::string& value) const {
  // Bouml preserved body begin 000B9BF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B9BF1
}

void ShortProtectionDisable::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B9C71
    message.stctOutput.typeNumber = 177;  //ShortProtectionDisable
    message.stctOutput.value = value;
  // Bouml preserved body end 000B9C71
}

void ShortProtectionDisable::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B9CF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B9CF1
}

SlopeControlHighSide::SlopeControlHighSide() {
  // Bouml preserved body begin 000B90F1
    this->name = "SlopeControlHighSide";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B90F1
}

SlopeControlHighSide::~SlopeControlHighSide() {
  // Bouml preserved body begin 000B9171
  // Bouml preserved body end 000B9171
}

void SlopeControlHighSide::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B91F1
    parameter = this->value;
  // Bouml preserved body end 000B91F1
}

void SlopeControlHighSide::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B9271
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B9271
}

void SlopeControlHighSide::toString(std::string& value) const {
  // Bouml preserved body begin 000B92F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B92F1
}

void SlopeControlHighSide::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B9371
    message.stctOutput.typeNumber = 175;  //SlopeControlHighSide
    message.stctOutput.value = value;
  // Bouml preserved body end 000B9371
}

void SlopeControlHighSide::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B93F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B93F1
}

SlopeControlLowSide::SlopeControlLowSide() {
  // Bouml preserved body begin 000B9571
    this->name = "SlopeControlLowSide";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B9571
}

SlopeControlLowSide::~SlopeControlLowSide() {
  // Bouml preserved body begin 000B95F1
  // Bouml preserved body end 000B95F1
}

void SlopeControlLowSide::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B9671
    parameter = this->value;
  // Bouml preserved body end 000B9671
}

void SlopeControlLowSide::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B96F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B96F1
}

void SlopeControlLowSide::toString(std::string& value) const {
  // Bouml preserved body begin 000B9771
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B9771
}

void SlopeControlLowSide::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B97F1
    message.stctOutput.typeNumber = 176;  //SlopeControlLowSide
    message.stctOutput.value = value;
  // Bouml preserved body end 000B97F1
}

void SlopeControlLowSide::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B9871
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B9871
}

SmartEnergyActualCurrent::SmartEnergyActualCurrent() {
  // Bouml preserved body begin 000BA771
    this->name = "SmartEnergyActualCurrent";
    this->lowerLimit = 0;
    this->upperLimit = 31;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BA771
}

SmartEnergyActualCurrent::~SmartEnergyActualCurrent() {
  // Bouml preserved body begin 000BA7F1
  // Bouml preserved body end 000BA7F1
}

void SmartEnergyActualCurrent::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BA871
    parameter = this->value;
  // Bouml preserved body end 000BA871
}

void SmartEnergyActualCurrent::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BA8F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BA8F1
}

void SmartEnergyActualCurrent::toString(std::string& value) const {
  // Bouml preserved body begin 000BA971
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BA971
}

void SmartEnergyActualCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BA9F1
    message.stctOutput.typeNumber = 180;   //SmartEnergyActualCurrent
    message.stctOutput.value = value;
  // Bouml preserved body end 000BA9F1
}

void SmartEnergyActualCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BAA71
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BAA71
}

SmartEnergyCurrentDownStep::SmartEnergyCurrentDownStep() {
  // Bouml preserved body begin 000B75F1
    this->name = "SmartEnergyCurrentDownStep";
    this->lowerLimit = 0;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B75F1
}

SmartEnergyCurrentDownStep::~SmartEnergyCurrentDownStep() {
  // Bouml preserved body begin 000B7671
  // Bouml preserved body end 000B7671
}

void SmartEnergyCurrentDownStep::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B76F1
    parameter = this->value;
  // Bouml preserved body end 000B76F1
}

void SmartEnergyCurrentDownStep::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B7771
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B7771
}

void SmartEnergyCurrentDownStep::toString(std::string& value) const {
  // Bouml preserved body begin 000B77F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B77F1
}

void SmartEnergyCurrentDownStep::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B7871
    message.stctOutput.typeNumber = 169;   //SmartEnergyCurrentDownStep
    message.stctOutput.value = value;
  // Bouml preserved body end 000B7871
}

void SmartEnergyCurrentDownStep::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B78F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B78F1
}

SmartEnergyCurrentMinimum::SmartEnergyCurrentMinimum() {
  // Bouml preserved body begin 000B7171
    this->name = "SmartEnergyCurrentMinimum";
    this->lowerLimit = 0;
    this->upperLimit = 1;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B7171
}

SmartEnergyCurrentMinimum::~SmartEnergyCurrentMinimum() {
  // Bouml preserved body begin 000B71F1
  // Bouml preserved body end 000B71F1
}

void SmartEnergyCurrentMinimum::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B7271
    parameter = this->value;
  // Bouml preserved body end 000B7271
}

void SmartEnergyCurrentMinimum::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B72F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B72F1
}

void SmartEnergyCurrentMinimum::toString(std::string& value) const {
  // Bouml preserved body begin 000B7371
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B7371
}

void SmartEnergyCurrentMinimum::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B73F1
    message.stctOutput.typeNumber = 168;  //SmartEnergyCurrentMinimum
    message.stctOutput.value = value;
  // Bouml preserved body end 000B73F1
}

void SmartEnergyCurrentMinimum::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B7471
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B7471
}

SmartEnergyCurrentUpStep::SmartEnergyCurrentUpStep() {
  // Bouml preserved body begin 000B7EF1
    this->name = "SmartEnergyCurrentUpStep";
    this->lowerLimit = 1;
    this->upperLimit = 3;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B7EF1
}

SmartEnergyCurrentUpStep::~SmartEnergyCurrentUpStep() {
  // Bouml preserved body begin 000B7F71
  // Bouml preserved body end 000B7F71
}

void SmartEnergyCurrentUpStep::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B7FF1
    parameter = this->value;
  // Bouml preserved body end 000B7FF1
}

void SmartEnergyCurrentUpStep::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B8071
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B8071
}

void SmartEnergyCurrentUpStep::toString(std::string& value) const {
  // Bouml preserved body begin 000B80F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B80F1
}

void SmartEnergyCurrentUpStep::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B8171
    message.stctOutput.typeNumber = 171;   //SmartEnergyCurrentUpStep
    message.stctOutput.value = value;
  // Bouml preserved body end 000B8171
}

void SmartEnergyCurrentUpStep::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B81F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B81F1
}

SmartEnergyHysteresis::SmartEnergyHysteresis() {
  // Bouml preserved body begin 000B7A71
    this->name = "SmartEnergyHysteresis";
    this->lowerLimit = 0;
    this->upperLimit = 15;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B7A71
}

SmartEnergyHysteresis::~SmartEnergyHysteresis() {
  // Bouml preserved body begin 000B7AF1
  // Bouml preserved body end 000B7AF1
}

void SmartEnergyHysteresis::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B7B71
    parameter = this->value;
  // Bouml preserved body end 000B7B71
}

void SmartEnergyHysteresis::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B7BF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B7BF1
}

void SmartEnergyHysteresis::toString(std::string& value) const {
  // Bouml preserved body begin 000B7C71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B7C71
}

void SmartEnergyHysteresis::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B7CF1
    message.stctOutput.typeNumber = 170;   //SmartEnergyHysteresis
    message.stctOutput.value = value;
  // Bouml preserved body end 000B7CF1
}

void SmartEnergyHysteresis::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B7D71
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B7D71
}

SmartEnergyHysteresisStart::SmartEnergyHysteresisStart() {
  // Bouml preserved body begin 000B8371
    this->name = "SmartEnergyHysteresisStart";
    this->lowerLimit = 0;
    this->upperLimit = 15;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B8371
}

SmartEnergyHysteresisStart::~SmartEnergyHysteresisStart() {
  // Bouml preserved body begin 000B83F1
  // Bouml preserved body end 000B83F1
}

void SmartEnergyHysteresisStart::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000B8471
    parameter = this->value;
  // Bouml preserved body end 000B8471
}

void SmartEnergyHysteresisStart::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000B84F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B84F1
}

void SmartEnergyHysteresisStart::toString(std::string& value) const {
  // Bouml preserved body begin 000B8571
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B8571
}

void SmartEnergyHysteresisStart::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B85F1
    message.stctOutput.typeNumber = 172;  //SmartEnergyHysteresisStart
    message.stctOutput.value = value;
  // Bouml preserved body end 000B85F1
}

void SmartEnergyHysteresisStart::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B8671
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B8671
}

SmartEnergySlowRunCurrent::SmartEnergySlowRunCurrent() {
  // Bouml preserved body begin 000BB4F1
    this->name = "SmartEnergySlowRunCurrent";
    this->lowerLimit = 0;
    this->upperLimit = 255;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BB4F1
}

SmartEnergySlowRunCurrent::~SmartEnergySlowRunCurrent() {
  // Bouml preserved body begin 000BB571
  // Bouml preserved body end 000BB571
}

void SmartEnergySlowRunCurrent::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BB5F1
    parameter = this->value;
  // Bouml preserved body end 000BB5F1
}

void SmartEnergySlowRunCurrent::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BB671
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BB671
}

void SmartEnergySlowRunCurrent::toString(std::string& value) const {
  // Bouml preserved body begin 000BB6F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BB6F1
}

void SmartEnergySlowRunCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BB771
    message.stctOutput.typeNumber = 183;   //SmartEnergySlowRunCurrent
    message.stctOutput.value = value;
  // Bouml preserved body end 000BB771
}

void SmartEnergySlowRunCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BB7F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BB7F1
}

SmartEnergyThresholdSpeed::SmartEnergyThresholdSpeed() {
  // Bouml preserved body begin 000BB071
    this->name = "SmartEnergyThresholdSpeed";
    this->lowerLimit = 0;
    this->upperLimit = 2047;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BB071
}

SmartEnergyThresholdSpeed::~SmartEnergyThresholdSpeed() {
  // Bouml preserved body begin 000BB0F1
  // Bouml preserved body end 000BB0F1
}

void SmartEnergyThresholdSpeed::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BB171
    parameter = this->value;
  // Bouml preserved body end 000BB171
}

void SmartEnergyThresholdSpeed::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BB1F1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BB1F1
}

void SmartEnergyThresholdSpeed::toString(std::string& value) const {
  // Bouml preserved body begin 000BB271
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BB271
}

void SmartEnergyThresholdSpeed::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BB2F1
    message.stctOutput.typeNumber = 182;  //SmartEnergyThresholdSpeed
    message.stctOutput.value = value;
  // Bouml preserved body end 000BB2F1
}

void SmartEnergyThresholdSpeed::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BB371
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BB371
}

StallGuard2FilterEnable::StallGuard2FilterEnable() {
  // Bouml preserved body begin 000B87F1
    this->name = "StallGuard2FilterEnable";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B87F1
}

StallGuard2FilterEnable::~StallGuard2FilterEnable() {
  // Bouml preserved body begin 000B8871
  // Bouml preserved body end 000B8871
}

void StallGuard2FilterEnable::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000B88F1
    parameter = this->value;
  // Bouml preserved body end 000B88F1
}

void StallGuard2FilterEnable::setParameter(const bool parameter) {
  // Bouml preserved body begin 000B8971
    this->value = parameter;
  // Bouml preserved body end 000B8971
}

void StallGuard2FilterEnable::toString(std::string& value) const {
  // Bouml preserved body begin 000B89F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B89F1
}

void StallGuard2FilterEnable::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B8A71
    message.stctOutput.typeNumber = 173;  //StallGuard2FilterEnable
    message.stctOutput.value = value;
  // Bouml preserved body end 000B8A71
}

void StallGuard2FilterEnable::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B8AF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B8AF1
}

StallGuard2Threshold::StallGuard2Threshold() {
  // Bouml preserved body begin 000B8C71
    this->name = "StallGuard2Threshold";
    this->lowerLimit = -64;
    this->upperLimit = 63;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B8C71
}

StallGuard2Threshold::~StallGuard2Threshold() {
  // Bouml preserved body begin 000B8CF1
  // Bouml preserved body end 000B8CF1
}

void StallGuard2Threshold::getParameter(int& parameter) const {
  // Bouml preserved body begin 000B8D71
    parameter = this->value;
  // Bouml preserved body end 000B8D71
}

void StallGuard2Threshold::setParameter(const int parameter) {
  // Bouml preserved body begin 000B8DF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B8DF1
}

void StallGuard2Threshold::toString(std::string& value) const {
  // Bouml preserved body begin 000B8E71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B8E71
}

void StallGuard2Threshold::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B8EF1
    message.stctOutput.typeNumber = 174;  //StallGuard2Threshold
    message.stctOutput.value = value;
  // Bouml preserved body end 000B8EF1
}

void StallGuard2Threshold::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B8F71
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 000B8F71
}

StandbyCurrent::StandbyCurrent() {
  // Bouml preserved body begin 000B3B71
    this->name = "StandbyCurrent";
    this->lowerLimit = 0;
    this->upperLimit = 255;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B3B71
}

StandbyCurrent::~StandbyCurrent() {
  // Bouml preserved body begin 000B3BF1
  // Bouml preserved body end 000B3BF1
}

void StandbyCurrent::getParameter(int& parameter) const {
  // Bouml preserved body begin 000B3C71
    parameter = this->value;
  // Bouml preserved body end 000B3C71
}

void StandbyCurrent::setParameter(const int parameter) {
  // Bouml preserved body begin 000B3CF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000B3CF1
}

void StandbyCurrent::toString(std::string& value) const {
  // Bouml preserved body begin 000B3D71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B3D71
}

void StandbyCurrent::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B3DF1
    message.stctOutput.typeNumber = 7;  //StandbyCurrent
    message.stctOutput.value = value;
  // Bouml preserved body end 000B3DF1
}

void StandbyCurrent::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B3E71
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B3E71
}

StepInterpolationEnable::StepInterpolationEnable() {
  // Bouml preserved body begin 000B51F1
    this->name = "StepInterpolationEnable";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000B51F1
}

StepInterpolationEnable::~StepInterpolationEnable() {
  // Bouml preserved body begin 000B5271
  // Bouml preserved body end 000B5271
}

void StepInterpolationEnable::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000B52F1
    parameter = this->value;
  // Bouml preserved body end 000B52F1
}

void StepInterpolationEnable::setParameter(const bool parameter) {
  // Bouml preserved body begin 000B5371
    this->value = parameter;
  // Bouml preserved body end 000B5371
}

void StepInterpolationEnable::toString(std::string& value) const {
  // Bouml preserved body begin 000B53F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000B53F1
}

void StepInterpolationEnable::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000B5471
    message.stctOutput.typeNumber = 160;  //StepInterpolationEnable
    message.stctOutput.value = value;
  // Bouml preserved body end 000B5471
}

void StepInterpolationEnable::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000B54F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000B54F1
}

StopOnStall::StopOnStall() {
  // Bouml preserved body begin 000BABF1
    this->name = "StopOnStall";
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BABF1
}

StopOnStall::~StopOnStall() {
  // Bouml preserved body begin 000BAC71
  // Bouml preserved body end 000BAC71
}

void StopOnStall::getParameter(bool& parameter) const {
  // Bouml preserved body begin 000BACF1
    parameter = this->value;
  // Bouml preserved body end 000BACF1
}

void StopOnStall::setParameter(const bool parameter) {
  // Bouml preserved body begin 000BAD71
    this->value = parameter;
  // Bouml preserved body end 000BAD71
}

void StopOnStall::toString(std::string& value) const {
  // Bouml preserved body begin 000BADF1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BADF1
}

void StopOnStall::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BAE71
    message.stctOutput.typeNumber = 181;  //StopOnStall
    if(value)
      message.stctOutput.value = 1;
    else
      message.stctOutput.value = 0;

  // Bouml preserved body end 000BAE71
}

void StopOnStall::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BAEF1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BAEF1
}

Vsense::Vsense() {
  // Bouml preserved body begin 000BA2F1
    this->name = "Vsense";
    this->lowerLimit = 0;
    this->upperLimit = 1;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 000BA2F1
}

Vsense::~Vsense() {
  // Bouml preserved body begin 000BA371
  // Bouml preserved body end 000BA371
}

void Vsense::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 000BA3F1
    parameter = this->value;
  // Bouml preserved body end 000BA3F1
}

void Vsense::setParameter(const unsigned int& parameter) {
  // Bouml preserved body begin 000BA471
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 000BA471
}

void Vsense::toString(std::string& value) const {
  // Bouml preserved body begin 000BA4F1
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 000BA4F1
}

void Vsense::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000BA571
    message.stctOutput.typeNumber = 179;  //Vsense
    message.stctOutput.value = value;
  // Bouml preserved body end 000BA571
}

void Vsense::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000BA5F1
    this->value = message.stctInput.value;
  // Bouml preserved body end 000BA5F1
}

ActualAcceleration::ActualAcceleration() {
  // Bouml preserved body begin 0010BBF1
    this->name = "ActualAcceleration";
    this->lowerLimit = 0;
    this->upperLimit = 2047;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 0010BBF1
}

ActualAcceleration::~ActualAcceleration() {
  // Bouml preserved body begin 0010BC71
  // Bouml preserved body end 0010BC71
}

void ActualAcceleration::getParameter(int& parameter) const {
  // Bouml preserved body begin 0010BCF1
    parameter = this->value;
  // Bouml preserved body end 0010BCF1
}

void ActualAcceleration::toString(std::string& value) const {
  // Bouml preserved body begin 0010BD71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 0010BD71
}

void ActualAcceleration::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 0010BDF1
    message.stctOutput.typeNumber = 135;  //ActualAcceleration
    message.stctOutput.value = value;
  // Bouml preserved body end 0010BDF1
}

void ActualAcceleration::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 0010BE71
    this->value = message.stctInput.value;
  // Bouml preserved body end 0010BE71
}

MinimumSpeed::MinimumSpeed() {
  // Bouml preserved body begin 00107B71
    this->name = "MinimumSpeed";
    this->lowerLimit = 0;
    this->upperLimit = 2047;
    this->parameterType = MOTOR_CONTOLLER_PARAMETER;
  // Bouml preserved body end 00107B71
}

MinimumSpeed::~MinimumSpeed() {
  // Bouml preserved body begin 00107BF1
  // Bouml preserved body end 00107BF1
}

void MinimumSpeed::getParameter(int& parameter) const {
  // Bouml preserved body begin 00107C71
    parameter = this->value;
  // Bouml preserved body end 00107C71
}

void MinimumSpeed::setParameter(const int parameter) {
  // Bouml preserved body begin 00107CF1
    if (this->lowerLimit > parameter) {
      throw std::out_of_range("The parameter exceeds the lower limit");
    }
    if (this->upperLimit < parameter) {
      throw std::out_of_range("The parameter exceeds the upper limit");
    }

    this->value = parameter;
  // Bouml preserved body end 00107CF1
}

void MinimumSpeed::toString(std::string& value) const {
  // Bouml preserved body begin 00107D71
  std::stringstream ss;
  ss << this->name << ": " << this->value;
  value  = ss.str();
  // Bouml preserved body end 00107D71
}

void MinimumSpeed::getYouBotMailboxMsg(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 00107DF1
    message.stctOutput.typeNumber = 130;  //MinimumSpeed
    message.stctOutput.value = (uint32)value;

  // Bouml preserved body end 00107DF1
}

void MinimumSpeed::setYouBotMailboxMsg(const YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 00107E71
    this->value = (int32)message.stctInput.value;
  // Bouml preserved body end 00107E71
}


} // namespace youbot
