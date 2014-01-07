#ifndef YOUBOT_WHEELEDBASEKINEMATIC_H
#define YOUBOT_WHEELEDBASEKINEMATIC_H

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
#include <vector>
#include <string>
#include "youbot_driver/generic/Units.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract class of a base / platform kinematic
///////////////////////////////////////////////////////////////////////////////
class BaseKinematic {
};
///////////////////////////////////////////////////////////////////////////////
/// abstract class of a wheeled based / platform kinematic
///////////////////////////////////////////////////////////////////////////////
class WheeledBaseKinematic : public BaseKinematic {
  public:
    virtual void cartesianVelocityToWheelVelocities(const quantity<si::velocity>& longitudinalVelocity, const quantity<si::velocity>& transversalVelocity, const quantity<angular_velocity>& angularVelocity, std::vector<quantity<angular_velocity> >& wheelVelocities) = 0;

    virtual void wheelVelocitiesToCartesianVelocity(const std::vector<quantity<angular_velocity> >& wheelVelocities, quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<angular_velocity>& angularVelocity) = 0;

    virtual void wheelPositionsToCartesianPosition(const std::vector<quantity<plane_angle> >& wheelPositions, quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation) = 0;

};

} // namespace youbot
#endif
