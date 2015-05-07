#ifndef YOUBOT_YOUBOTMANIPULATOR_H
#define YOUBOT_YOUBOTMANIPULATOR_H

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
#include "youbot_driver/youbot/YouBotGripper.hpp"
#include "youbot_driver/youbot/YouBotJoint.hpp"
#include "youbot_driver/youbot/EthercatMasterInterface.hpp"
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// It groups the manipulator joints and the gripper together
///////////////////////////////////////////////////////////////////////////////
class YouBotManipulator {
  public:
    YouBotManipulator(const std::string name, const std::string configFilePath = "../config/");

    virtual ~YouBotManipulator();

    ///does the sine commutation of the arm joints
    void doJointCommutation();

    ///calibrate the reference position of the arm joints
    void calibrateManipulator(const bool forceCalibration = false);

    void calibrateGripper(const bool forceCalibration = false);

    int getNumberJoints();

    bool hasGripper();

    ///return a joint form the arm1
    ///@param armJointNumber 1-5 for the arm joints
    YouBotJoint& getArmJoint(const unsigned int armJointNumber);

    YouBotGripper& getArmGripper();

    ///commands positions or angles to all manipulator joints
    ///all positions will be set at the same time
    ///@param JointData the to command positions
    virtual void setJointData(const std::vector<JointAngleSetpoint>& JointData);

    ///gets the position or angle of all manipulator joints which have been calculated from the actual encoder value
    ///These values are all read at the same time from the different joints 
    ///@param data returns the angles by reference
    virtual void getJointData(std::vector<JointSensedAngle>& data);

    ///commands velocities to all manipulator joints
    ///all velocities will be set at the same time
    ///@param JointData the to command velocities
    virtual void setJointData(const std::vector<JointVelocitySetpoint>& JointData);

    ///gets the velocities of all manipulator joints which have been calculated from the actual encoder values
    ///These values are all read at the same time from the different joints 
    ///@param data returns the velocities by reference
    virtual void getJointData(std::vector<JointSensedVelocity>& data);

    ///commands current to all manipulator joints
    ///all current values will be set at the same time
    ///@param JointData the to command current
    virtual void setJointData(const std::vector<JointCurrentSetpoint>& JointData);

    ///gets the motor currents of all manipulator joints which have been measured by a hal sensor
    ///These values are all read at the same time from the different joints 
    ///@param data returns the actual motor currents by reference
    virtual void getJointData(std::vector<JointSensedCurrent>& data);

    ///commands torque to all manipulator joints
    ///all torque values will be set at the same time
    ///@param JointData the to command torque 
    virtual void setJointData(const std::vector<JointTorqueSetpoint>& JointData);

    ///gets the joint torque of all manipulator joints which have been calculated from the current
    ///These values are all read at the same time from the different joints 
    ///@param data returns the actual joint torque by reference
    virtual void getJointData(std::vector<JointSensedTorque>& data);


  private:
    YouBotManipulator(const YouBotManipulator & source);

    YouBotManipulator & operator=(const YouBotManipulator & source);

    ///does the commutation of the arm joints with firmware 2.0
    void commutationFirmware200();

    ///does the commutation of the arm joints with firmware 1.48 and below
    void commutationFirmware148();

    void initializeJoints();

    unsigned int numberArmJoints; /// The number of manipulator joints

    boost::scoped_ptr<ConfigFile> configfile;

    boost::ptr_vector<YouBotJoint> joints;

    boost::scoped_ptr<YouBotGripper> gripper;

    int controllerType;

    EthercatMasterInterface& ethercatMaster;

    bool useGripper;
    bool isGripperInitialized;

    EthercatMasterWithThread* ethercatMasterWithThread;

    int alternativeControllerType;

    std::vector<std::string> supportedFirmwareVersions;

    std::string actualFirmwareVersionAllJoints;

};

} // namespace youbot
#endif
