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
#include "youbot/DataTrace.hpp"
namespace youbot {

  DataTrace::DataTrace(YouBotJoint& youBotJoint) : joint(youBotJoint) {
    // Bouml preserved body begin 000C8F71

    roundsPerMinuteSetpoint.rpm = 0;
    PWMSetpoint.pwm = 0;
    encoderSetpoint.encoderTicks = 0;
    // Bouml preserved body end 000C8F71
  }

  DataTrace::~DataTrace() {
    // Bouml preserved body begin 000C8FF1
    // Bouml preserved body end 000C8FF1
  }

  void DataTrace::startTrace() {
    // Bouml preserved body begin 000C93F1


    file.open("jointDataTrace", std::fstream::out | std::fstream::trunc);

    file << "# time [milliseconds]"
            << " " << "angle setpoint [rad]"
            << " " << "velocity setpoint [rad/s]"
            << " " << "RPM setpoint"
            << " " << "current setpoint [A]"
            << " " << "torque setpoint [Nm]"
            << " " << "PWM setpoint"
            << " " << "encoder setpoint"

            << " " << "sensed angle [rad]"
            << " " << "sensed encoder ticks"
            << " " << "sensed velocity [rad/s]"
            << " " << "sensed RPM"
            << " " << "sensed current [A]"
            << " " << "sensed torque [Nm]"

            << " " << "OVER_CURRENT" << " "
            << "UNDER_VOLTAGE" << " "
            << "OVER_VOLTAGE" << " "
            << "OVER_TEMPERATURE" << " "
            << "MOTOR_HALTED" << " "
            << "HALL_SENSOR_ERROR" << " "
            << "ENCODER_ERROR" << " "
            << "INITIALIZATION_ERROR" << " "
            << "PWM_MODE_ACTIVE" << " "
            << "VELOCITY_MODE" << " "
            << "POSITION_MODE" << " "
            << "TORQUE_MODE" << " "
            << "EMERGENCY_STOP" << " "
            << "FREERUNNING" << " "
            << "POSITION_REACHED" << " "
            << "INITIALIZED" << " "
            << "TIMEOUT" << " "
            << "I2T_EXCEEDED" << std::endl;

    traceStartTime = microsec_clock::local_time();
    // Bouml preserved body end 000C93F1
  }

  void DataTrace::stopTrace() {
    // Bouml preserved body begin 000C9471
    file.close();
    // Bouml preserved body end 000C9471
  }

  void DataTrace::plotTrace() {
    // Bouml preserved body begin 000C9571
    std::system("gnuplot ../gnuplotconfig");
    // Bouml preserved body end 000C9571
  }

  void DataTrace::updateTrace(const JointAngleSetpoint& setpoint) {
    // Bouml preserved body begin 000C9071
    angleSetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C9071
  }

  void DataTrace::updateTrace(const JointVelocitySetpoint& setpoint) {
    // Bouml preserved body begin 000C90F1
    velocitySetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C90F1
  }

  void DataTrace::updateTrace(const JointRoundsPerMinuteSetpoint& setpoint) {
    // Bouml preserved body begin 000C9171
    roundsPerMinuteSetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C9171
  }

  void DataTrace::updateTrace(const JointCurrentSetpoint& setpoint) {
    // Bouml preserved body begin 000C91F1
    currentSetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C91F1
  }

  void DataTrace::updateTrace(const JointTorqueSetpoint& setpoint) {
    // Bouml preserved body begin 000C9271
    torqueSetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C9271
  }

  void DataTrace::updateTrace(const JointPWMSetpoint& setpoint) {
    // Bouml preserved body begin 000C92F1
    PWMSetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C92F1
  }

  void DataTrace::updateTrace(const JointEncoderSetpoint& setpoint) {
    // Bouml preserved body begin 000C9371
    encoderSetpoint = setpoint;
    this->update();
    // Bouml preserved body end 000C9371
  }

  void DataTrace::update() {
    // Bouml preserved body begin 000C94F1
    timeDuration = microsec_clock::local_time() - traceStartTime;
    timeDurationMicroSec = timeDuration.total_milliseconds();
    unsigned short statusFlags;

    joint.getStatus(statusFlags);
    joint.getData(sensedAngle);
    joint.getData(sensedEncoderTicks);
    joint.getData(sensedVelocity);
    joint.getData(sensedRoundsPerMinute);
    joint.getData(sensedCurrent);
    joint.getData(sensedTorque);

    file << timeDurationMicroSec //1
            << " " << angleSetpoint.angle.value() //2
            << " " << velocitySetpoint.angularVelocity.value() //3
            << " " << roundsPerMinuteSetpoint.rpm //4
            << " " << currentSetpoint.current.value() //5
            << " " << torqueSetpoint.torque.value() //6
            << " " << PWMSetpoint.pwm //7
            << " " << encoderSetpoint.encoderTicks //8

            << " " << sensedAngle.angle.value() //9
            << " " << sensedEncoderTicks.encoderTicks //10
            << " " << sensedVelocity.angularVelocity.value() //11
            << " " << sensedRoundsPerMinute.rpm //12
            << " " << sensedCurrent.current.value() //13
            << " " << sensedTorque.torque.value() //14

            << " " << bool(statusFlags & OVER_CURRENT) << " " //15
            << bool(statusFlags & UNDER_VOLTAGE) << " " //16
            << bool(statusFlags & OVER_VOLTAGE) << " " //17
            << bool(statusFlags & OVER_TEMPERATURE) << " " //18
            << bool(statusFlags & MOTOR_HALTED) << " " //19
            << bool(statusFlags & HALL_SENSOR_ERROR) << " " //20
            << bool(statusFlags & ENCODER_ERROR) << " " //21
            << bool(statusFlags & INITIALIZATION_ERROR) << " " //22
            << bool(statusFlags & PWM_MODE_ACTIVE) << " " //23
            << bool(statusFlags & VELOCITY_MODE) << " " //24
            << bool(statusFlags & POSITION_MODE) << " " //25
            << bool(statusFlags & TORQUE_MODE) << " " //26
            << bool(statusFlags & EMERGENCY_STOP) << " " //27
            << bool(statusFlags & FREERUNNING) << " " //28
            << bool(statusFlags & POSITION_REACHED) << " " //29
            << bool(statusFlags & INITIALIZED) << " " //30
            << bool(statusFlags & TIMEOUT) << " " //31
            << bool(statusFlags & I2T_EXCEEDED) << std::endl; //32


    // Bouml preserved body end 000C94F1
  }


} // namespace youbot
