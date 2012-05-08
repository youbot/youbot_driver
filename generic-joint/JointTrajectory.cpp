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
#include "generic-joint/JointTrajectory.hpp"
namespace youbot {

void JointTrajectory::setTrajectoryPoint(const quantity<plane_angle>& position, const quantity<angular_velocity>& velocity, const quantity<angular_acceleration>& acceleration, const quantity<si::time>& time) {
  // Bouml preserved body begin 000F5B71
	this->positions.push_back(position);
	this->velocities.push_back(velocity);
	this->accelerations.push_back(acceleration);
	this->time.push_back(time);
  // Bouml preserved body end 000F5B71
}

const std::vector< quantity<plane_angle> >& JointTrajectory::getPositions() {
  // Bouml preserved body begin 000F5BF1
	return this->positions;
  // Bouml preserved body end 000F5BF1
}

const std::vector< quantity<angular_velocity> >& JointTrajectory::getVelocities() {
  // Bouml preserved body begin 000F5C71
	return this->velocities;
  // Bouml preserved body end 000F5C71
}

const std::vector< quantity<angular_acceleration> >& JointTrajectory::getAccelerations() {
  // Bouml preserved body begin 000F5CF1
	return this->accelerations;
  // Bouml preserved body end 000F5CF1
}

const std::vector<quantity<si::time> >& JointTrajectory::getTimes() {
  // Bouml preserved body begin 000F5D71
	return this->time;
  // Bouml preserved body end 000F5D71
}


} // namespace youbot
