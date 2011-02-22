#ifndef YOUBOT_YOUBOTBASE_H
#define YOUBOT_YOUBOTBASE_H

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
#include <sstream>
#include <string>
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/ConfigFile.hpp"
#include "generic/Exceptions.hpp"
#include "youbot/YouBotJoint.hpp"
#include "youbot/EthercatMaster.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
#include "base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// It groups the base joints together
///////////////////////////////////////////////////////////////////////////////
class YouBotBase {
  public:
    YouBotBase(const std::string name, const std::string configFilePath = "../config/");

    virtual ~YouBotBase();

    ///return a joint form the base
    ///@param baseJointNumber 1-4 for the base joints
    YouBotJoint& getBaseJoint(const unsigned int baseJointNumber);

    ///gets the cartesien base position which is calculated from the odometry
    ///@param longitudinalPosition is the forward or backward position
    ///@param transversalPosition is the sideway position
    ///@param orientation is the rotation around the center of the YouBot
    void getBasePosition(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation);

    ///gets the cartesien base velocity
    ///@param longitudinalVelocity is the forward or backward velocity
    ///@param transversalVelocity is the sideway velocity
    ///@param angularVelocity is the rotational velocity around the center of the YouBot
    void getBaseVelocity(quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<si::angular_velocity>& angularVelocity);

    ///commands the base in cartesien velocities
    ///@param longitudinalVelocity is the forward or backward velocity
    ///@param transversalVelocity is the sideway velocity
    ///@param angularVelocity is the rotational velocity around the center of the YouBot
    void setBaseVelocity(const quantity<si::velocity>& longitudinalVelocity, const quantity<si::velocity>& transversalVelocity, const quantity<si::angular_velocity>& angularVelocity);

    ///This class represents the kinematic of the YouBot 
    FourSwedishWheelOmniBaseKinematic youBotBaseKinematic;


  private:
    void initializeJoints();

    void initializeKinematic();

    void doJointCommutation();

    ConfigFile* configfile;

    std::vector<YouBotJoint> joints;

    std::string configFilePath;

    std::string ethercatConfigFileName;

};

} // namespace youbot
#endif
