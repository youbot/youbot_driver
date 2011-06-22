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

    this->controllerType = 841;
    this->minFirmwareVersion = 1.43;

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
    if (configfile != NULL)
      delete configfile;
  // Bouml preserved body end 00067FF1
}

///does the sine commutation of the arm joints
void YouBotManipulator::doJointCommutation() {
  // Bouml preserved body begin 000A3371

    InitializeJoint doInitialization;
    bool isInitialized = false;
    int noInitialization = 0;
    std::string jointName;


    ClearMotorControllerTimeoutFlag clearTimeoutFlag;
    for (unsigned int i = 1; i <= ARMJOINTS; i++) {
      this->getArmJoint(i).setConfigurationParameter(clearTimeoutFlag);
    }

    for (unsigned int i = 1; i <= ARMJOINTS; i++) {
      doInitialization.setParameter(false);
      this->getArmJoint(i).getConfigurationParameter(doInitialization);
      doInitialization.getParameter(isInitialized);
      if (!isInitialized) {
        noInitialization++;
      }
    }

    if (noInitialization != 0) {
      LOG(info) << "Manipulator Joint Commutation";
      doInitialization.setParameter(true);

      EthercatMaster::getInstance().AutomaticReceiveOn(false);
      this->getArmJoint(1).setConfigurationParameter(doInitialization);
      this->getArmJoint(2).setConfigurationParameter(doInitialization);
      this->getArmJoint(3).setConfigurationParameter(doInitialization);
      this->getArmJoint(4).setConfigurationParameter(doInitialization);
      this->getArmJoint(5).setConfigurationParameter(doInitialization);
      EthercatMaster::getInstance().AutomaticReceiveOn(true);

      unsigned short statusFlags;
      std::vector<bool> isCommutated;
      isCommutated.assign(ARMJOINTS, false);
      unsigned int u = 0;

      // check for the next 5 sec if the joints are commutated
      for (u = 1; u <= 5000; u++) {
        for (unsigned int i = 1; i <= ARMJOINTS; i++) {
          this->getArmJoint(i).getStatus(statusFlags);
          if (statusFlags & INITIALIZED) {
            isCommutated[i - 1] = true;
          }
        }
        if (isCommutated[0] && isCommutated[1] && isCommutated[2] && isCommutated[3] && isCommutated[4]) {
          break;
        }
        SLEEP_MILLISEC(1);
      }

      SLEEP_MILLISEC(10); // the controller likes it

      for (unsigned int i = 1; i <= ARMJOINTS; i++) {
        doInitialization.setParameter(false);
        this->getArmJoint(i).getConfigurationParameter(doInitialization);
        doInitialization.getParameter(isInitialized);
        if (!isInitialized) {
          std::stringstream jointNameStream;
          jointNameStream << "Joint " << i;
          jointName = jointNameStream.str();
          throw std::runtime_error("could not commutation " + jointName);
        }
      }
    }


  // Bouml preserved body end 000A3371
}

///calibrate the reference position of the arm joints
void YouBotManipulator::calibrateManipulator(const bool forceCalibration) {
  // Bouml preserved body begin 000A9C71

    //Calibrate all manipulator joints
    std::vector<JointRoundsPerMinuteSetpoint> calibrationVel;
    JointRoundsPerMinuteSetpoint tempdummy;
    tempdummy.rpm = 0;
    calibrationVel.assign(ARMJOINTS, tempdummy);
    std::vector<quantity<si::current> > maxCurrent;
    quantity<si::current> tempdummy2;
    maxCurrent.assign(ARMJOINTS, tempdummy2);
    std::vector<bool> doCalibration;
    doCalibration.assign(ARMJOINTS, true);
    std::string jointName;

    double dummy = 0;
    char index = 56; // up to 55 is user variable

    //56 is the last user variable at bank 2
    YouBotSlaveMailboxMsg IsCalibratedReadMessage;
    IsCalibratedReadMessage.stctOutput.moduleAddress = DRIVE;
    IsCalibratedReadMessage.stctOutput.commandNumber = GGP;
    IsCalibratedReadMessage.stctOutput.typeNumber = index;
    IsCalibratedReadMessage.stctOutput.motorNumber = USER_VARIABLE_BANK;
    IsCalibratedReadMessage.stctOutput.value = 0;
    IsCalibratedReadMessage.stctInput.value = 0;

    YouBotSlaveMailboxMsg IsCalibratedSetMessage;
    IsCalibratedSetMessage.stctOutput.moduleAddress = DRIVE;
    IsCalibratedSetMessage.stctOutput.commandNumber = SGP;
    IsCalibratedSetMessage.stctOutput.typeNumber = index;
    IsCalibratedSetMessage.stctOutput.motorNumber = USER_VARIABLE_BANK;
    IsCalibratedSetMessage.stctOutput.value = 1;


    //get parameters for calibration
    for (unsigned int i = 0; i < ARMJOINTS; i++) {

      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();
      bool calib = true;
      configfile->readInto(calib, jointName, "DoCalibration");
      doCalibration[i] = calib;

      joints[i].getConfigurationParameter(IsCalibratedReadMessage);
      if (IsCalibratedReadMessage.stctInput.value == 1) {
        doCalibration[i] = false;
      }

      if (forceCalibration) {
        doCalibration[i] = true;
      }

      configfile->readInto(dummy, jointName, "CalibrationMaxCurrent_[ampere]");
      maxCurrent[i] = dummy * ampere;
      std::string direction;
      configfile->readInto(direction, jointName, "CalibrationDirection");
      GearRatio gearRatio;
      joints[i].getConfigurationParameter(gearRatio);
      double gearratio = 1;
      gearRatio.getParameter(gearratio);

      if (direction == "POSITIV") {
        calibrationVel[i].rpm = 1 / gearratio;
      } else if (direction == "NEGATIV") {
        calibrationVel[i].rpm = -1 / gearratio;
      } else {
        throw std::runtime_error("Wrong calibration direction for " + jointName);
      }
    }


    LOG(info) << "Calibrate Joints ";

    std::vector<bool> finished;
    finished.assign(ARMJOINTS, false);
    JointSensedCurrent sensedCurrent;
    JointRoundsPerMinuteSetpoint stopMovement;
    stopMovement.rpm = 0;


    //move the joints slowly in calibration direction
    for (unsigned int i = 0; i < ARMJOINTS; i++) {
      if (doCalibration[i] == true) {
        joints[i].setData(calibrationVel[i]);
      } else {
        finished[i] = true;
      }
    }

    //monitor the current to find end stop 
    while (!(finished[0] && finished[1] && finished[2] && finished[3] && finished[4])) {
      for (unsigned int i = 0; i < ARMJOINTS; i++) {
        joints[i].getData(sensedCurrent);
        //turn till a max current is reached
        if (abs(sensedCurrent.current) > abs(maxCurrent[i])) {
          //stop movement
          joints[i].setData(stopMovement);
          finished[i] = true;
        }
      }
      SLEEP_MILLISEC(1);
    }

    // wait to let the joint stop the motion
    SLEEP_MILLISEC(500);

    for (unsigned int i = 0; i < ARMJOINTS; i++) {
      if (doCalibration[i] == true) {
        //set encoder reference position
        joints[i].setEncoderToZero();

        /*
        //switch to position controll
        SLEEP_MILLISEC(100);
        messageBuffer.stctOutput.controllerMode = POSITION_CONTROL;
        messageBuffer.stctOutput.positionOrSpeed = 0;
        //   LOG(trace) << "vel [rpm] " << messageBuffer.stctOutput.positionOrSpeed << " rad_sec " << data.angularVelocity;
        EthercatMaster::getInstance().setMsgBuffer(messageBuffer, this->jointNumber);
         */
        // set a flag in the user variable to remember that it is calibrated
        joints[i].setConfigurationParameter(IsCalibratedSetMessage);

        //     LOG(info) << "Calibration finished for joint: " << this->jointName;
      }
    }

    //  if(doCalibration[0] && doCalibration[1] && doCalibration[2] && doCalibration[3] && doCalibration[4] ){
//    JointAngleSetpoint desiredJointAngle;
//
//    desiredJointAngle.angle = 2.56244 * radian;
//    joints[0].setData(desiredJointAngle);
//
//    desiredJointAngle.angle = 1.04883 * radian;
//    joints[1].setData(desiredJointAngle);
//
//    desiredJointAngle.angle = -2.43523 * radian;
//    joints[2].setData(desiredJointAngle);
//
//    desiredJointAngle.angle = 1.73184 * radian;
//    joints[3].setData(desiredJointAngle);
//
//    desiredJointAngle.angle = 1.73184 * radian;
//    joints[4].setData(desiredJointAngle);
//    LOG(info) << "unfold arm";
//    SLEEP_MILLISEC(4000);
    //   }

    //setting joint Limits
    JointLimits jLimits;
    for (unsigned int i = 0; i < ARMJOINTS; i++) {
      long upperlimit = 0, lowerlimit = 0;
      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();
      JointEncoderSetpoint minEncoderValue;
      configfile->readInto(lowerlimit, jointName, "LowerLimit_[encoderTicks]");
      configfile->readInto(upperlimit, jointName, "UpperLimit_[encoderTicks]");
      minEncoderValue.encoderTicks = upperlimit;

      jLimits.setParameter(lowerlimit, upperlimit, true);
      joints[i].setConfigurationParameter(jLimits);
    }



  // Bouml preserved body end 000A9C71
}

void YouBotManipulator::calibrateGripper() {
  // Bouml preserved body begin 000A9CF1
    // Calibrating Gripper
    bool doCalibration = true;
    configfile->readInto(doCalibration, "Gripper", "DoCalibration");
    CalibrateGripper calibrate;
    calibrate.setParameter(doCalibration);
    gripperVector[0].setConfigurationParameter(calibrate);
  // Bouml preserved body end 000A9CF1
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
///@param JointData the to command positions
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
///@param JointData the to command velocities
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

bool YouBotManipulator::areSame(const double A, const double B) {
  // Bouml preserved body begin 000A82F1
    return std::fabs(A - B) < 0.0001;
  // Bouml preserved body end 000A82F1
}

void YouBotManipulator::initializeJoints() {
  // Bouml preserved body begin 00068071

    LOG(info) << "Initializing Joints";


    //get number of slaves
    unsigned int noSlaves = EthercatMaster::getInstance(this->ethercatConfigFileName, this->configFilePath).getNumberOfSlaves();


    if (noSlaves < ARMJOINTS) {
      throw std::runtime_error("Not enough ethercat slaves were found to create a YouBotManipulator!");
    }

    // configfile.setSection("JointTopology");

    unsigned int slaveNumber = 0;
    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint1");
    if (slaveNumber <= noSlaves) {
      joints.push_back(YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint2");
    if (slaveNumber <= noSlaves) {
      joints.push_back(YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint3");
    if (slaveNumber <= noSlaves) {
      joints.push_back(YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint4");
    if (slaveNumber <= noSlaves) {
      joints.push_back(YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint5");
    if (slaveNumber <= noSlaves) {
      joints.push_back(YouBotJoint(slaveNumber));
    } else {
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
    contollerGearRatio.setParameter(0);
    FirmwareVersion firmwareTypeVersion;



    for (unsigned int i = 0; i < ARMJOINTS; i++) {
      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();


      joints[i].getConfigurationParameter(firmwareTypeVersion);
      std::string version;
      int controllerType;
      double firmwareVersion;
      firmwareTypeVersion.getParameter(controllerType, firmwareVersion);

      string name;
      configfile->readInto(name, jointName, "JointName");
      jName.setParameter(name);

      LOG(info) << jointName << " " << name << ": Controller Type: " << controllerType << " Firmware version: " << firmwareVersion;

      if (this->controllerType != controllerType) {
        std::stringstream ss;
        ss << "The youBot manipulator motor controller have to be of type: " << this->controllerType;
        throw std::runtime_error(ss.str().c_str());
      }

      if (!areSame(firmwareVersion, this->minFirmwareVersion)) {
        if (firmwareVersion < this->minFirmwareVersion) {
          std::stringstream ss;
          ss << "The motor controller firmware version have be " << this->minFirmwareVersion << " or higher.";
          throw std::runtime_error(ss.str().c_str());
        }
      }

      //check if the motor contoller gear ratio is one.
      //The gear ratio will be taken in to acount by the driver
      joints[i].getConfigurationParameter(contollerGearRatio);
      unsigned int cGearRatio;
      contollerGearRatio.getParameter(cGearRatio);
      if (cGearRatio != 1) {
        throw std::runtime_error("The Motor Contoller Gear Ratio of " + jointName + " is not set to 1.");
      }

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


    //Initializing Gripper
    // configfile.setSection("JointTopology");
    configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint5");
    this->gripperVector.push_back(YouBotGripper(slaveNumber));
    BarSpacingOffset barOffest;
    MaxTravelDistance maxDistance;
    MaxEncoderValue maxEncoder;
    double dummy = 0;

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


    return;
  // Bouml preserved body end 00068071
}


} // namespace youbot
