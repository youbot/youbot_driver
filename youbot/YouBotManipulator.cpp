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
#include "youbot/YouBotManipulator.hpp"
namespace youbot {

YouBotManipulator::YouBotManipulator(const std::string name, const std::string configFilePath) {
  // Bouml preserved body begin 00067F71

  string filename;
  filename = name;
  filename.append(".cfg");

  this->configFilePath = configFilePath;
  this->ethercatConfigFileName = "youbot-ethercat.cfg";

  configfile == NULL;
  configfile = new ConfigFile(filename, configFilePath);

  this->initializeJoints();

  // Bouml preserved body end 00067F71
}

YouBotManipulator::~YouBotManipulator() {
  // Bouml preserved body begin 00067FF1
  if(configfile != NULL)
    delete configfile;
  // Bouml preserved body end 00067FF1
}

///return a joint form the arm1
///@param armJointNumber 1-5 for the arm1 joints
YouBotJoint& YouBotManipulator::getArmJoint(const unsigned int armJointNumber) {
  // Bouml preserved body begin 0004F7F1

    if (armJointNumber <= 0 || armJointNumber > ARMJOINTS) {
      throw std::out_of_range("Invalid Joint Number");
    }
    return joints[armJointNumber - 1];
  // Bouml preserved body end 0004F7F1
}

YouBotGripper& YouBotManipulator::getArmGripper() {
  // Bouml preserved body begin 0005F9F1
    if (this->gripperVector.size() >= 1) {
      return this->gripperVector[0];
    } else {
      throw std::out_of_range("There is no Gripper");
    }
  // Bouml preserved body end 0005F9F1
}

///commands positions or angles to all manipulator joints
///all positions will be set at the same time
///@param data the to command positions
void YouBotManipulator::setJointData(const std::vector<JointAngleSetpoint>& JointData) {
  // Bouml preserved body begin 0008FDF1
    if (JointData.size() != ARMJOINTS)
      throw std::out_of_range("Wrong number of JointAngleSetpoints");

    EthercatMaster::getInstance().AutomaticSendOn(false);
    joints[0].setData(JointData[0], NON_BLOCKING);
    joints[1].setData(JointData[1], NON_BLOCKING);
    joints[2].setData(JointData[2], NON_BLOCKING);
    joints[3].setData(JointData[3], NON_BLOCKING);
    joints[4].setData(JointData[4], NON_BLOCKING);
    EthercatMaster::getInstance().AutomaticSendOn(true);

  // Bouml preserved body end 0008FDF1
}

///gets the position or angle of all manipulator joints which have been calculated from the actual encoder value
///These values are all read at the same time from the different joints 
///@param data returns the angles by reference
void YouBotManipulator::getJointData(std::vector<JointSensedAngle>& data) {
  // Bouml preserved body begin 0008FE71
    data.resize(ARMJOINTS);
    EthercatMaster::getInstance().AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    joints[4].getData(data[4]);
    EthercatMaster::getInstance().AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FE71
}

///commands velocities to all manipulator joints
///all velocities will be set at the same time
///@param data the to command velocities
void YouBotManipulator::setJointData(const std::vector<JointVelocitySetpoint>& JointData) {
  // Bouml preserved body begin 0008FEF1
    if (JointData.size() != ARMJOINTS)
      throw std::out_of_range("Wrong number of JointVelocitySetpoints");

    EthercatMaster::getInstance().AutomaticSendOn(false);
    joints[0].setData(JointData[0], NON_BLOCKING);
    joints[1].setData(JointData[1], NON_BLOCKING);
    joints[2].setData(JointData[2], NON_BLOCKING);
    joints[3].setData(JointData[3], NON_BLOCKING);
    joints[4].setData(JointData[4], NON_BLOCKING);
    EthercatMaster::getInstance().AutomaticSendOn(true);
  // Bouml preserved body end 0008FEF1
}

///gets the velocities of all manipulator joints which have been calculated from the actual encoder values
///These values are all read at the same time from the different joints 
///@param data returns the velocities by reference
void YouBotManipulator::getJointData(std::vector<JointSensedVelocity>& data) {
  // Bouml preserved body begin 0008FF71
    data.resize(ARMJOINTS);
    EthercatMaster::getInstance().AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    joints[4].getData(data[4]);
    EthercatMaster::getInstance().AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FF71
}

///gets temperatures of all manipulator motors which have been measured by a thermometer
///These values are all read at the same time from the different joints 
///@param data returns the actual temperatures by reference
void YouBotManipulator::getJointData(std::vector<JointSensedTemperature>& data) {
  // Bouml preserved body begin 0008FFF1
    data.resize(ARMJOINTS);
    EthercatMaster::getInstance().AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    joints[4].getData(data[4]);
    EthercatMaster::getInstance().AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FFF1
}

///gets the motor currents of all manipulator joints which have been measured by a hal sensor
///These values are all read at the same time from the different joints 
///@param data returns the actual motor currents by reference
void YouBotManipulator::getJointData(std::vector<JointSensedCurrent>& data) {
  // Bouml preserved body begin 00090071
    data.resize(ARMJOINTS);
    EthercatMaster::getInstance().AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    joints[4].getData(data[4]);
    EthercatMaster::getInstance().AutomaticReceiveOn(true);
  // Bouml preserved body end 00090071
}

void YouBotManipulator::initializeJoints() {
  // Bouml preserved body begin 00068071

    LOG(info) << "Initializing Joints";


    //get number of slaves
    unsigned int noSlaves = EthercatMaster::getInstance(this->ethercatConfigFileName, this->configFilePath).getNumberOfSlaves();


    if(noSlaves < ARMJOINTS){
      throw std::runtime_error("Not enough ethercat slaves were found to create a YouBotManipulator!");
    }

   // configfile.setSection("JointTopology");

    unsigned int slaveNumber = 0;
    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint1");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint2");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber,"JointTopology", "ManipulatorJoint3");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber,"JointTopology", "ManipulatorJoint4");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber,"JointTopology", "ManipulatorJoint5");
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
    double gearRatio_numerator = 0;
    double gearRatio_denominator = 1;
    MotorContollerGearRatio contollerGearRatio;
    contollerGearRatio.setParameter(1);


    for (unsigned int i = 0; i < ARMJOINTS; i++) {
      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();

      //set the motor contoller gear ratio to one.
      //The gear ratio will be taken in to acount by the driver
      joints[i].setConfigurationParameter(contollerGearRatio);

      string name;
      configfile->readInto(name, jointName, "JointName");
      jName.setParameter(name);
      configfile->readInto(gearRatio_numerator, jointName, "GearRatio_numerator");
      configfile->readInto(gearRatio_denominator, jointName, "GearRatio_denominator");
      gearRatio.setParameter(gearRatio_numerator / gearRatio_denominator);
      int ticks;
      configfile->readInto(ticks, jointName, "EncoderTicksPerRound");
      ticksPerRound.setParameter(ticks);
      bool invdir = false;
      configfile->readInto(invdir, jointName, "InverseMovementDirection");
      inverseDir.setParameter(invdir);

      joints[i].setConfigurationParameter(jName);
      joints[i].setConfigurationParameter(gearRatio);
      joints[i].setConfigurationParameter(ticksPerRound);
      joints[i].setConfigurationParameter(inverseDir);
    }


    //TODO When to calibrate the manipulator and when it is not necessary
    //Calibrate all manipulator joints
    std::vector<CalibrateJoint> calibrateJointVec;
    quantity<si::current> current;
    JointLimits jLimits;
    double dummy = 0;
    bool doCalibration = true;

    for (unsigned int i = 0; i < ARMJOINTS; i++) {

      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();
 //     configfile.setSection(jointName.c_str());

      configfile->readInto(doCalibration, jointName, "DoCalibration");

      int upperlimit = 0, lowerlimit = 0;
      configfile->readInto(lowerlimit, jointName, "LowerLimit_[encoderTicks]");
      configfile->readInto(upperlimit, jointName, "UpperLimit_[encoderTicks]");

      jLimits.setParameter(lowerlimit, upperlimit, true);
      joints[i].setConfigurationParameter(jLimits);
      
      configfile->readInto(dummy, jointName, "CalibrationMaxCurrent_[ampere]");
      current = dummy * ampere;
      std::string direction;
      configfile->readInto(direction, jointName, "CalibrationDirection");

      calibrateJointVec.push_back(CalibrateJoint());

      if (direction == "POSITIV") {
        calibrateJointVec[i].setParameter(doCalibration, POSITIV, current);
      } else if (direction == "NEGATIV") {
        calibrateJointVec[i].setParameter(doCalibration, NEGATIV, current);
      } else {
        throw std::runtime_error("Wrong calibration direction for " + jointName);
      }
      joints[i].setConfigurationParameter(calibrateJointVec[i]);
    }



    //Initializing Gripper
   // configfile.setSection("JointTopology");
    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint5");
    this->gripperVector.push_back(YouBotGripper(slaveNumber));
    BarSpacingOffset barOffest;
    MaxTravelDistance maxDistance;
    MaxEncoderValue maxEncoder;

 //   configfile.setSection("Gripper");
    configfile->readInto(doCalibration, "Gripper", "DoCalibration");
    configfile->readInto(dummy, "Gripper", "BarSpacingOffset_[meter]");
    barOffest.setParameter(dummy * meter);
    gripperVector[0].setConfigurationParameter(barOffest);
    configfile->readInto(dummy, "Gripper", "MaxTravelDistance_[meter]");
    maxDistance.setParameter(dummy * meter);
    gripperVector[0].setConfigurationParameter(maxDistance);
    int maxenc = 0;
    configfile->readInto(maxenc, "Gripper", "MaxEncoderValue");
    maxEncoder.setParameter(maxenc);
    gripperVector[0].setConfigurationParameter(maxEncoder);

    // Calibrating Gripper
    CalibrateGripper calibrate;
    calibrate.setParameter(doCalibration);
    gripperVector[0].setConfigurationParameter(calibrate);

    return;
  // Bouml preserved body end 00068071
}


} // namespace youbot
