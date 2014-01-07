#ifndef YOUBOT_ONEDOFGRIPPERDATA_H
#define YOUBOT_ONEDOFGRIPPERDATA_H

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
#include "youbot_driver/generic-gripper/GripperData.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract class of data for gripper with one degree of freedom
///////////////////////////////////////////////////////////////////////////////
class OneDOFGripperData : public GripperData {
};
///////////////////////////////////////////////////////////////////////////////
/// Setpoint length of the bar spacing for a one DOF gripper
///////////////////////////////////////////////////////////////////////////////
class GripperBarSpacingSetPoint : public OneDOFGripperData {
  public:
    quantity<si::length> barSpacing;

};
///////////////////////////////////////////////////////////////////////////////
/// The sensed bar spacing for a one DOF gripper
///////////////////////////////////////////////////////////////////////////////
class GripperSensedBarSpacing : public OneDOFGripperData {
  public:
    quantity<si::length> barSpacing;

};
///////////////////////////////////////////////////////////////////////////////
/// The sensed bar velocity for a one DOF gripper
///////////////////////////////////////////////////////////////////////////////
class GripperSensedVelocity : public OneDOFGripperData {
  public:
    long barVelocity;

};
///////////////////////////////////////////////////////////////////////////////
/// The encoder setpoint for one bar
///////////////////////////////////////////////////////////////////////////////
class GripperBarEncoderSetpoint : public OneDOFGripperData {
  public:
    int barEncoder;

};
///////////////////////////////////////////////////////////////////////////////
/// The bar position for a one gripper bar
///////////////////////////////////////////////////////////////////////////////
class GripperBarPositionSetPoint : public OneDOFGripperData {
  public:
    quantity<si::length> barPosition;

};
///////////////////////////////////////////////////////////////////////////////
/// The sensed bar position for a one gripper bar
///////////////////////////////////////////////////////////////////////////////
class GripperSensedBarPosition : public OneDOFGripperData {
  public:
    quantity<si::length> barPosition;

};

} // namespace youbot
#endif
