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
#include "generic/Units.hpp"

namespace youbot {

class JointData {
};
class JointSensedData : public JointData {
};
class JointSensedAngle : public JointSensedData {
  public:
    quantity<plane_angle> angle;

};
class JointSensedVelocity : public JointSensedData {
  public:
    quantity<si::angular_velocity> angularVelocity;

};
class JointSensedCurrent : public JointSensedData {
  public:
    quantity<si::current> current;

};
class JointSensedTorque : public JointSensedData {
  public:
    quantity<si::torque> torque;

};
class JointSensedTemperature : public JointSensedData {
  public:
    quantity<celsius::temperature> temperature;

};
class JointDataSetpoint : public JointData {
};
class JointAngleSetpoint : public JointDataSetpoint {
  public:
    quantity<plane_angle> angle;

};
class JointVelocitySetpoint : public JointDataSetpoint {
  public:
    quantity<angular_velocity> angularVelocity;

};
class JointCurrentSetpoint : public JointDataSetpoint {
  public:
    quantity<si::current> current;

};
class JointTorqueSetpoint : public JointDataSetpoint {
  public:
    quantity<si::torque> torque;

};

} // namespace youbot
#endif
