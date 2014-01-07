#ifndef YOUBOT_JOINTDATA_H
#define YOUBOT_JOINTDATA_H

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
#include "youbot_driver/generic/Units.hpp"

namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract data class for joints
///////////////////////////////////////////////////////////////////////////////
class JointData {
};
///////////////////////////////////////////////////////////////////////////////
/// abstract data class for sensed / measured joint data
///////////////////////////////////////////////////////////////////////////////
class JointSensedData : public JointData {
};
///////////////////////////////////////////////////////////////////////////////
/// abstract data class for computed joint data
///////////////////////////////////////////////////////////////////////////////
class JointComputedData : public JointSensedData {
};
///////////////////////////////////////////////////////////////////////////////
/// actual pwm value of the joint
///////////////////////////////////////////////////////////////////////////////
class JointSensedPWM : public JointSensedData {
  public:
    signed int pwm;

    JointSensedPWM();

    JointSensedPWM(const signed int& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Sensed position / angle of the joint
///////////////////////////////////////////////////////////////////////////////
class JointSensedAngle : public JointSensedData {
  public:
    quantity<plane_angle> angle;

    JointSensedAngle();

    JointSensedAngle(const quantity<plane_angle>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Sensed encoder ticks of the joint
///////////////////////////////////////////////////////////////////////////////
class JointSensedEncoderTicks : public JointSensedData {
  public:
    signed int encoderTicks;

    JointSensedEncoderTicks();

    JointSensedEncoderTicks(const signed int& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Sensed velocity of the joint
///////////////////////////////////////////////////////////////////////////////
class JointSensedVelocity : public JointSensedData {
  public:
    quantity<si::angular_velocity> angularVelocity;

    JointSensedVelocity();

    JointSensedVelocity(const quantity<si::angular_velocity>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Sensed rounds per minute (rpm) of the joint
///////////////////////////////////////////////////////////////////////////////
class JointSensedRoundsPerMinute : public JointSensedData {
  public:
    int rpm;

    JointSensedRoundsPerMinute();

    JointSensedRoundsPerMinute(const int value);

};
///////////////////////////////////////////////////////////////////////////////
/// Sensed electric current of the joint
///////////////////////////////////////////////////////////////////////////////
class JointSensedCurrent : public JointSensedData {
  public:
    quantity<si::current> current;

    JointSensedCurrent();

    JointSensedCurrent(const quantity<si::current>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// This torque of the joint is computed from the actual current
///////////////////////////////////////////////////////////////////////////////
class JointSensedTorque : public JointComputedData {
  public:
    quantity<si::torque> torque;

    JointSensedTorque();

    JointSensedTorque(const quantity<si::torque>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// abstract data class for commanded joint data
///////////////////////////////////////////////////////////////////////////////
class JointDataSetpoint : public JointData {
};
///////////////////////////////////////////////////////////////////////////////
/// abstract data class for computed joint data setpoints
///////////////////////////////////////////////////////////////////////////////
class JointComputedSetpoint : public JointDataSetpoint {
};
///////////////////////////////////////////////////////////////////////////////
/// Set-point angle / position of the joint
///////////////////////////////////////////////////////////////////////////////
class JointAngleSetpoint : public JointDataSetpoint {
  public:
    quantity<plane_angle> angle;

    JointAngleSetpoint();

    JointAngleSetpoint(const quantity<plane_angle>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Set-point velocity of the joint
///////////////////////////////////////////////////////////////////////////////
class JointVelocitySetpoint : public JointDataSetpoint {
  public:
    quantity<angular_velocity> angularVelocity;

    JointVelocitySetpoint();

    JointVelocitySetpoint(const quantity<angular_velocity>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Rounds per minute set-point of the joint
///////////////////////////////////////////////////////////////////////////////
class JointRoundsPerMinuteSetpoint : public JointDataSetpoint {
  public:
    int rpm;

    JointRoundsPerMinuteSetpoint();

    JointRoundsPerMinuteSetpoint(const int value);

};
///////////////////////////////////////////////////////////////////////////////
/// Set-point current of the joint
///////////////////////////////////////////////////////////////////////////////
class JointCurrentSetpoint : public JointDataSetpoint {
  public:
    quantity<si::current> current;

    JointCurrentSetpoint();

    JointCurrentSetpoint(const quantity<si::current>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// The torque set-point of the joint will be set by setting the computed current set-point
///////////////////////////////////////////////////////////////////////////////
class JointTorqueSetpoint : public JointComputedSetpoint {
  public:
    quantity<si::torque> torque;

    JointTorqueSetpoint();

    JointTorqueSetpoint(const quantity<si::torque>& value);

};
///////////////////////////////////////////////////////////////////////////////
/// Pulse-width modulation set-point of the joint
///////////////////////////////////////////////////////////////////////////////
class JointPWMSetpoint : public JointDataSetpoint {
  public:
    int pwm;

    JointPWMSetpoint();

    JointPWMSetpoint(const int value);

};
///////////////////////////////////////////////////////////////////////////////
/// encoder ticks setpoint of the joint
///////////////////////////////////////////////////////////////////////////////
class JointEncoderSetpoint : public JointDataSetpoint {
  public:
    signed int encoderTicks;

    JointEncoderSetpoint();

    JointEncoderSetpoint(const signed int& value);

};
///////////////////////////////////////////////////////////////////////////////
/// abstract data class for data which have been computed by the hardware controller
///////////////////////////////////////////////////////////////////////////////
class JointControllerComputedData : public JointSensedData {
};
///////////////////////////////////////////////////////////////////////////////
/// Sensed velocity of the joint
///////////////////////////////////////////////////////////////////////////////
class JointRampGeneratorVelocity : public JointControllerComputedData {
  public:
    quantity<si::angular_velocity> angularVelocity;

    JointRampGeneratorVelocity();

    JointRampGeneratorVelocity(const quantity<si::angular_velocity>& value);

};

} // namespace youbot
#endif
