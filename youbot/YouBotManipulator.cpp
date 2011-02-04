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
  filename = configFilePath;
  filename.append(name);
  filename.append(".cfg");

  this->configFilePath = configFilePath;
  this->ethercatConfigFileName = "youbot-ethercat.cfg";

  configfile == NULL;
  configfile = new ConfigFile(filename.c_str());

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

    if (armJointNumber <= 0 || armJointNumber > 5) {
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

void YouBotManipulator::initializeJoints() {
  // Bouml preserved body begin 00068071

    LOG(info) << "Initializing Joints";


    //get number of slaves
    unsigned int noSlaves = EthercatMaster::getInstance(this->ethercatConfigFileName, this->configFilePath).getNumberOfSlaves();


    if(noSlaves < 5){
      throw std::runtime_error("Not enough ethercat slaves were found to create a YouBotManipulator!");
    }

   // configfile.setSection("JointTopology");

    unsigned int slaveNumber = 0;
    configfile->readInto(slaveNumber, "ManipulatorJoint1");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "ManipulatorJoint2");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "ManipulatorJoint3");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "ManipulatorJoint4");
    if(slaveNumber  <= noSlaves){
      joints.push_back(YouBotJoint(slaveNumber));
    }else{
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "ManipulatorJoint5");
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


    for (unsigned int i = 0; i < 5; i++) {
      std::stringstream jointNameStream;
      jointNameStream << "J" << i + 1;
      jointName = jointNameStream.str();
    //  configfile.setSection(jointName.c_str());

      //set the motor contoller gear ratio to one.
      //The gear ratio will be taken in to acount by the driver
      joints[i].setConfigurationParameter(contollerGearRatio);

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
    }

   
    //TODO When to calibrate the manipulator and when it is not necessary
    //Calibrate all manipulator joints
    std::vector<CalibrateJoint> calibrateJointVec;
    quantity<si::current> current;
    JointLimits jLimits;
    double dummy = 0;
    bool doCalibration = true;

    for (unsigned int i = 0; i < 5; i++) {

      std::stringstream jointNameStream;
      jointNameStream << "J" << i + 1;
      jointName = jointNameStream.str();
 //     configfile.setSection(jointName.c_str());

      configfile->readInto(doCalibration, jointName+"DoCalibration");

      int upperlimit = 0, lowerlimit = 0;
      configfile->readInto(lowerlimit, jointName+"LowerLimit_[encoderTicks]");
      configfile->readInto(upperlimit, jointName+"UpperLimit_[encoderTicks]");

      jLimits.setParameter(lowerlimit, upperlimit, true);
      joints[i].setConfigurationParameter(jLimits);
      
      configfile->readInto(dummy, jointName+"CalibrationMaxCurrent_[ampere]");
      current = dummy * ampere;
      std::string direction;
      configfile->readInto(direction, jointName+"CalibrationDirection");

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
    configfile->readInto(slaveNumber, "ManipulatorJoint5");
    this->gripperVector.push_back(YouBotGripper(slaveNumber));
    BarSpacingOffset barOffest;
    MaxTravelDistance maxDistance;
    MaxEncoderValue maxEncoder;

 //   configfile.setSection("Gripper");
    configfile->readInto(doCalibration, "DoCalibration");
    configfile->readInto(dummy, "BarSpacingOffset_[meter]");
    barOffest.setParameter(dummy * meter);
    gripperVector[0].setConfigurationParameter(barOffest);
    configfile->readInto(dummy, "MaxTravelDistance_[meter]");
    maxDistance.setParameter(dummy * meter);
    gripperVector[0].setConfigurationParameter(maxDistance);
    int maxenc = 0;
    configfile->readInto(maxenc, "MaxEncoderValue");
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
