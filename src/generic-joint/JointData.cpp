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
#include "youbot_driver/generic-joint/JointData.hpp"

namespace youbot {

JointSensedPWM::JointSensedPWM() {
  // Bouml preserved body begin 000CD971
  this->pwm = 0;
  // Bouml preserved body end 000CD971
}

JointSensedPWM::JointSensedPWM(const signed int& value) {
  // Bouml preserved body begin 000CFDF1
  this->pwm = value;
  // Bouml preserved body end 000CFDF1
}

JointSensedAngle::JointSensedAngle() {
  // Bouml preserved body begin 000CD771
  this->angle = 0 * radian;
  // Bouml preserved body end 000CD771
}

JointSensedAngle::JointSensedAngle(const quantity<plane_angle>& value) {
  // Bouml preserved body begin 000CFBF1
  this->angle = value;
  // Bouml preserved body end 000CFBF1
}

JointSensedEncoderTicks::JointSensedEncoderTicks() {
  // Bouml preserved body begin 000CD7F1
  this->encoderTicks = 0;
  // Bouml preserved body end 000CD7F1
}

JointSensedEncoderTicks::JointSensedEncoderTicks(const signed int& value) {
  // Bouml preserved body begin 000CFC71
  this->encoderTicks = value;
  // Bouml preserved body end 000CFC71
}

JointSensedVelocity::JointSensedVelocity() {
  // Bouml preserved body begin 000CD6F1
  this->angularVelocity = 0 * radian_per_second;
  // Bouml preserved body end 000CD6F1
}

JointSensedVelocity::JointSensedVelocity(const quantity<si::angular_velocity>& value) {
  // Bouml preserved body begin 000CFB71
  this->angularVelocity = value;
  // Bouml preserved body end 000CFB71
}

JointSensedRoundsPerMinute::JointSensedRoundsPerMinute() {
  // Bouml preserved body begin 000CD871
  this->rpm = 0;
  // Bouml preserved body end 000CD871
}

JointSensedRoundsPerMinute::JointSensedRoundsPerMinute(const int value) {
  // Bouml preserved body begin 000CFCF1
  this->rpm = value;
  // Bouml preserved body end 000CFCF1
}

JointSensedCurrent::JointSensedCurrent() {
  // Bouml preserved body begin 000CD8F1
  this->current = 0 * ampere;
  // Bouml preserved body end 000CD8F1
}

JointSensedCurrent::JointSensedCurrent(const quantity<si::current>& value) {
  // Bouml preserved body begin 000CFD71
  this->current = value;
  // Bouml preserved body end 000CFD71
}

JointSensedTorque::JointSensedTorque() {
  // Bouml preserved body begin 000CD671
  this->torque = 0 * newton_meter;
  // Bouml preserved body end 000CD671
}

JointSensedTorque::JointSensedTorque(const quantity<si::torque>& value) {
  // Bouml preserved body begin 000CFAF1
  this->torque = value;
  // Bouml preserved body end 000CFAF1
}

JointAngleSetpoint::JointAngleSetpoint() {
  // Bouml preserved body begin 000CDB71
  this->angle = 0 * radian;
  // Bouml preserved body end 000CDB71
}

JointAngleSetpoint::JointAngleSetpoint(const quantity<plane_angle>& value) {
  // Bouml preserved body begin 000CFF71
  this->angle = value;
  // Bouml preserved body end 000CFF71
}

JointVelocitySetpoint::JointVelocitySetpoint() {
  // Bouml preserved body begin 000CDAF1
  this->angularVelocity = 0 * radian_per_second;
  // Bouml preserved body end 000CDAF1
}

JointVelocitySetpoint::JointVelocitySetpoint(const quantity<angular_velocity>& value) {
  // Bouml preserved body begin 000CFEF1
  this->angularVelocity = value;
  // Bouml preserved body end 000CFEF1
}

JointRoundsPerMinuteSetpoint::JointRoundsPerMinuteSetpoint() {
  // Bouml preserved body begin 000CDCF1
  this->rpm = 0;
  // Bouml preserved body end 000CDCF1
}

JointRoundsPerMinuteSetpoint::JointRoundsPerMinuteSetpoint(const int value) {
  // Bouml preserved body begin 000D00F1
  this->rpm = value;
  // Bouml preserved body end 000D00F1
}

JointCurrentSetpoint::JointCurrentSetpoint() {
  // Bouml preserved body begin 000CDBF1
  this->current = 0 * ampere;
  // Bouml preserved body end 000CDBF1
}

JointCurrentSetpoint::JointCurrentSetpoint(const quantity<si::current>& value) {
  // Bouml preserved body begin 000CFFF1
  this->current = value;
  // Bouml preserved body end 000CFFF1
}

JointTorqueSetpoint::JointTorqueSetpoint() {
  // Bouml preserved body begin 000CDA71
  this->torque = 0 * newton_meter;
  // Bouml preserved body end 000CDA71
}

JointTorqueSetpoint::JointTorqueSetpoint(const quantity<si::torque>& value) {
  // Bouml preserved body begin 000CFE71
  this->torque = value;
  // Bouml preserved body end 000CFE71
}

JointPWMSetpoint::JointPWMSetpoint() {
  // Bouml preserved body begin 000CDC71
  this->pwm = 0;
  // Bouml preserved body end 000CDC71
}

JointPWMSetpoint::JointPWMSetpoint(const int value) {
  // Bouml preserved body begin 000D0071
  this->pwm = value;
  // Bouml preserved body end 000D0071
}

JointEncoderSetpoint::JointEncoderSetpoint() {
  // Bouml preserved body begin 000CDD71
  this->encoderTicks = 0;
  // Bouml preserved body end 000CDD71
}

JointEncoderSetpoint::JointEncoderSetpoint(const signed int& value) {
  // Bouml preserved body begin 000D0171
  this->encoderTicks = value;
  // Bouml preserved body end 000D0171
}

JointRampGeneratorVelocity::JointRampGeneratorVelocity() {
  // Bouml preserved body begin 00103DF1
  this->angularVelocity = 0 * radian_per_second;
  // Bouml preserved body end 00103DF1
}

JointRampGeneratorVelocity::JointRampGeneratorVelocity(const quantity<si::angular_velocity>& value) {
  // Bouml preserved body begin 00103E71
  this->angularVelocity = value;
  // Bouml preserved body end 00103E71
}


} // namespace youbot
