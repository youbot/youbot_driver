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
#include "youbot/YouBotBase.hpp"
namespace youbot {

YouBotBase::YouBotBase(const std::string name, const std::string configFilePath) {
  // Bouml preserved body begin 00067E71

  string filename;
  filename = configFilePath;
  filename.append(name);
  filename.append(".cfg");
  configfile == NULL;

  this->configFilePath = configFilePath;
  this->ethercatConfigFileName = "youbot-ethercat.cfg";

  configfile = new ConfigFile(filename.c_str());



  this->initializeJoints();

  this->initializeKinematic();

  // Bouml preserved body end 00067E71
}

YouBotBase::~YouBotBase() {
  // Bouml preserved body begin 00067EF1
  if(configfile != NULL)
    delete configfile;
  // Bouml preserved body end 00067EF1
}

///return a joint form the base
///@param baseJointNumber 1-4 for the base joints
YouBotJoint& YouBotBase::getBaseJoint(const unsigned int baseJointNumber) {
  // Bouml preserved body begin 0004F771
    if (baseJointNumber <= 0 || baseJointNumber > 4 ) {
      throw std::out_of_range("Invalid Joint Number");
    }
    return joints[baseJointNumber - 1];
  // Bouml preserved body end 0004F771
}

///gets the cartesien base position which is calculated from the odometry
///@param longitudinalPosition is the forward or backward position
///@param transversalPosition is the sideway position
///@param orientation is the rotation around the center of the YouBot
void YouBotBase::getBasePosition(quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation) {
  // Bouml preserved body begin 000514F1

    std::vector<quantity<plane_angle> > wheelPositions;
    quantity<plane_angle> dummy;
    JointSensedAngle sensedPos;
    wheelPositions.assign(4, dummy);

    joints[0].getData(sensedPos);
    wheelPositions[0] = sensedPos.angle;
    joints[1].getData(sensedPos);
    wheelPositions[1] = sensedPos.angle;
    joints[2].getData(sensedPos);
    wheelPositions[2] = sensedPos.angle;
    joints[3].getData(sensedPos);
    wheelPositions[3] = sensedPos.angle;

    youBotBaseKinematic.wheelPositionsToCartesianPosition(wheelPositions, longitudinalPosition, transversalPosition, orientation);
  // Bouml preserved body end 000514F1
}

///gets the cartesien base velocity
///@param longitudinalVelocity is the forward or backward velocity
///@param transversalVelocity is the sideway velocity
///@param angularVelocity is the rotational velocity around the center of the YouBot
void YouBotBase::getBaseVelocity(quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<si::angular_velocity>& angularVelocity) {
  // Bouml preserved body begin 00051271

    std::vector<quantity<angular_velocity> > wheelVelocities;
    quantity<angular_velocity> dummy;
    JointSensedVelocity sensedVel;
    wheelVelocities.assign(4, dummy);

    joints[0].getData(sensedVel);
    wheelVelocities[0] = sensedVel.angularVelocity;
    joints[1].getData(sensedVel);
    wheelVelocities[1] = sensedVel.angularVelocity;
    joints[2].getData(sensedVel);
    wheelVelocities[2] = sensedVel.angularVelocity;
    joints[3].getData(sensedVel);
    wheelVelocities[3] = sensedVel.angularVelocity;

    youBotBaseKinematic.wheelVelocitiesToCartesianVelocity(wheelVelocities, longitudinalVelocity, transversalVelocity, angularVelocity);

  // Bouml preserved body end 00051271
}

///commands the base in cartesien velocities
///@param longitudinalVelocity is the forward or backward velocity
///@param transversalVelocity is the sideway velocity
///@param angularVelocity is the rotational velocity around the center of the YouBot
void YouBotBase::setBaseVelocity(const quantity<si::velocity>& longitudinalVelocity, const quantity<si::velocity>& transversalVelocity, const quantity<si::angular_velocity>& angularVelocity) {
  // Bouml preserved body begin 0004DD71

    std::vector<quantity<angular_velocity> > wheelVelocities;
    JointVelocitySetpoint setVel;

    youBotBaseKinematic.cartesianVelocityToWheelVelocities(longitudinalVelocity, transversalVelocity, angularVelocity, wheelVelocities);

    if (wheelVelocities.size() < 4)
      throw std::out_of_range("To less wheel velocities");

    EthercatMaster::getInstance().AutomaticSendOn(false);
    setVel.angularVelocity = wheelVelocities[0];
    joints[0].setData(setVel, NON_BLOCKING);
    setVel.angularVelocity = wheelVelocities[1];
    joints[1].setData(setVel, NON_BLOCKING);
    setVel.angularVelocity = wheelVelocities[2];
    joints[2].setData(setVel, NON_BLOCKING);
    setVel.angularVelocity = wheelVelocities[3];
    joints[3].setData(setVel, NON_BLOCKING);
    EthercatMaster::getInstance().AutomaticSendOn(true);
  // Bouml preserved body end 0004DD71
}

void YouBotBase::initializeJoints() {
  // Bouml preserved body begin 000464F1

    LOG(info) << "Initializing Joints";

    //get number of slaves
    unsigned int noSlaves = EthercatMaster::getInstance(this->ethercatConfigFileName, this->configFilePath).getNumberOfSlaves();

    if(noSlaves < 4){
      throw std::out_of_range("Not enough ethercat slaves were found to create a YouBotBase!");
    }
    
    //read Joint Topology from config file
  //  configfile.setSection("JointTopology");

    //check if enough slave exist to create YouBotJoint for the slave numbers from config file
    //if enough slave exist create YouBotJoint and store it in the joints vector
    unsigned int slaveNumber = 0;
    configfile->readInto(slaveNumber, "BaseLeftFront");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "BaseRightFront");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "BaseLeftBack");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "BaseRightBack");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }


    //Configure Joint Parameters
    std::string jointName;
    JointName jName;
    GearRatio gearRatio;
    EncoderTicksPerRound ticksPerRound;
    InverseMovementDirection inverseDir;
    MotorContollerGearRatio contollerGearRatio;
    contollerGearRatio.setParameter(1);

    double gearRatio_numerator = 0;
    double gearRatio_denominator = 1;

    for (unsigned int i = 0; i < 4; i++) {
      std::stringstream jointNameStream;
      jointNameStream << "J" << i + 1;
      jointName = jointNameStream.str();
    //  configfile.setSection(jointName.c_str());

      std:string name;
      configfile->readInto(name, jointName+"JointName");
      jName.setParameter(name);
      configfile->readInto(gearRatio_numerator, jointName+"GearRatio_numerator");
      configfile->readInto(gearRatio_denominator, jointName+"GearRatio_denominator");
      gearRatio.setParameter(gearRatio_numerator / gearRatio_denominator);
      int ticks;
      configfile->readInto(ticks, jointName+"EncoderTicksPerRound");
      ticksPerRound.setParameter(ticks);
      bool invdir = false;
      configfile->readInto(invdir, jointName+"InverseMovementDirection");
      inverseDir.setParameter(invdir);

      joints[i].setConfigurationParameter(jName);
      joints[i].setConfigurationParameter(gearRatio);
      joints[i].setConfigurationParameter(ticksPerRound);
      joints[i].setConfigurationParameter(inverseDir);

      //set the motor contoller gear ratio to one.
      //The gear ratio will be taken in to acount by the driver
      joints[i].setConfigurationParameter(contollerGearRatio);
    }

    return;
  // Bouml preserved body end 000464F1
}

void YouBotBase::initializeKinematic() {
  // Bouml preserved body begin 0004DDF1
    FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;

    //read the kinematics parameter from a config file
  //  configfile.setSection("YouBotKinematic");
    configfile->readInto(kinematicConfig.rotationRatio, "RotationRatio");
    configfile->readInto(kinematicConfig.slideRatio, "SlideRatio");
    double dummy = 0;
    configfile->readInto(dummy, "LengthBetweenFrontAndRearWheels_[meter]");
    kinematicConfig.lengthBetweenFrontAndRearWheels = dummy * meter;
    configfile->readInto(dummy, "LengthBetweenFrontWheels_[meter]");
    kinematicConfig.lengthBetweenFrontWheels = dummy * meter;
    configfile->readInto(dummy, "WheelRadius_[meter]");
    kinematicConfig.wheelRadius = dummy * meter;
    

    youBotBaseKinematic.setConfiguration(kinematicConfig);
  // Bouml preserved body end 0004DDF1
}


} // namespace youbot
