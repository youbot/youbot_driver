#ifndef YOUBOT_JOINTTRAJECTORYCONTROLLER_H
#define YOUBOT_JOINTTRAJECTORYCONTROLLER_H

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
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/Exceptions.hpp"
#include "youbot/YouBotJoint.hpp"
#include "youbot/YouBotJointParameter.hpp"
#include "youbot/YouBotJointParameterPasswordProtected.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// 
///////////////////////////////////////////////////////////////////////////////
class JointTrajectoryController {
  public:
    JointTrajectoryController(YouBotJoint& joint);

    virtual ~JointTrajectoryController();

    /// calculates all trajectory values for the future and sets all the the ethercat master
    /// if the trajectory is still active the the values in the next buffer
    void setTrajectory(const std::vector< quantity<plane_angle> >& positions, const std::vector< quantity<angular_velocity> >& velocities, const std::vector< quantity<angular_acceleration> >& accelerations);

    /// Stops just the trajectory controller but not the joint movement
    void stopTrajectory();

    /// Stops the joint movement by decreasing the velocity until zero
    void stopJointMovement();

    /// returns true if the trajectory controller is active
    bool isTrajectoryControllerActive();


  private:
    void calculateVelocities(const quantity<plane_angle>& position_delta, const quantity<angular_velocity>& velocity, const quantity<angular_velocity>& velocity_current, const quantity<angular_acceleration>& acceleration, std::list<int32>& targetVelocities);

    YouBotJoint& joint;

    double gearRatio;

    EthercatMasterWithThread* ethercatMaster;

};

} // namespace youbot
#endif
