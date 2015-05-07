
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
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"
namespace youbot {

YouBotManipulator::YouBotManipulator(const std::string name, const std::string configFilePath)
: ethercatMaster(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath)) {
  // Bouml preserved body begin 00067F71

    this->controllerType = 841;
    this->alternativeControllerType = 1610;
    this->supportedFirmwareVersions.push_back("148");
    this->supportedFirmwareVersions.push_back("200");
    this->actualFirmwareVersionAllJoints = "";
	this->numberArmJoints = 5;

    string filename;
    filename = name;
    filename.append(".cfg");
	useGripper = true;
    isGripperInitialized = false;
    configfile.reset(new ConfigFile(filename, configFilePath));
		
		if(ethercatMaster.isThreadActive()){
			ethercatMasterWithThread = static_cast<EthercatMasterWithThread*>(&(EthercatMaster::getInstance()));
		}else{
			ethercatMasterWithThread = NULL;
		}

    this->initializeJoints();
    


  // Bouml preserved body end 00067F71
}

YouBotManipulator::~YouBotManipulator() {
  // Bouml preserved body begin 00067FF1
		if(ethercatMaster.isThreadActive()){
			for (unsigned int i = 0; i < numberArmJoints; i++) {
				ethercatMasterWithThread->deleteJointTrajectoryControllerRegistration(this->joints[i].getJointNumber());
			}
		}
  // Bouml preserved body end 00067FF1
}

///does the sine commutation of the arm joints
void YouBotManipulator::doJointCommutation() {
  // Bouml preserved body begin 000A3371

  if(this->actualFirmwareVersionAllJoints == "148"){
    this->commutationFirmware148();
  }else if(this->actualFirmwareVersionAllJoints == "200" ){
    this->commutationFirmware200();
  }else{
    throw std::runtime_error("Unable to commutate joints - Unsupported firmware version!");
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
    calibrationVel.assign(numberArmJoints, tempdummy);
    std::vector<quantity<si::current> > maxCurrent;
    quantity<si::current> tempdummy2;
    maxCurrent.assign(numberArmJoints, tempdummy2);
    std::vector<bool> doCalibration;
    doCalibration.assign(numberArmJoints, true);
    std::string jointName;

    double dummy = 0;
    char index = 16; // Parameter 0 to 15 of bank 2 are password protected

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
    for (unsigned int i = 0; i < numberArmJoints; i++) {

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


    LOG(info) << "Calibrate Manipulator Joints ";

    std::vector<bool> finished;
    finished.assign(numberArmJoints, false);
    int numberUnfinished = numberArmJoints;
    JointSensedCurrent sensedCurrent;


    //move the joints slowly in calibration direction
    for (unsigned int i = 0; i < numberArmJoints; i++) {
      if (doCalibration[i] == true) {
        joints[i].setData(calibrationVel[i]);
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
      } else {
        if (!finished[i]) {
            finished[i] = true;
            numberUnfinished--;
        }
      }
    }

    //monitor the current to find end stop 
    while (numberUnfinished > 0) {
      for (unsigned int i = 0; i < numberArmJoints; i++) {
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
        joints[i].getData(sensedCurrent);
        //turn till a max current is reached
        if (abs(sensedCurrent.current) > abs(maxCurrent[i])) {
          //stop movement
          youbot::JointCurrentSetpoint currentStopMovement;
          currentStopMovement.current = 0 * ampere;
          joints[i].setData(currentStopMovement);
          if(!ethercatMaster.isThreadActive()){
            ethercatMaster.sendProcessData();
            ethercatMaster.receiveProcessData();
          }
          if (!finished[i]) {
              finished[i] = true;
              numberUnfinished--;
          }
        }
      }
      SLEEP_MILLISEC(1);
    }

    // wait to let the joint stop the motion
    SLEEP_MILLISEC(100);

    for (unsigned int i = 0; i < numberArmJoints; i++) {
      if (doCalibration[i] == true) {
        //set encoder reference position
        joints[i].setEncoderToZero();
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
        // set a flag in the user variable to remember that it is calibrated
        joints[i].setConfigurationParameter(IsCalibratedSetMessage);
        //     LOG(info) << "Calibration finished for joint: " << this->jointName;
      }
    }

    //setting joint Limits
    JointLimits jLimits;
    for (unsigned int i = 0; i < numberArmJoints; i++) {
      long upperlimit = 0, lowerlimit = 0;
      std::stringstream jointNameStream;
      bool inverted = false;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();
      JointEncoderSetpoint minEncoderValue;
      configfile->readInto(lowerlimit, jointName, "LowerLimit_[encoderTicks]");
      configfile->readInto(upperlimit, jointName, "UpperLimit_[encoderTicks]");
      configfile->readInto(inverted, jointName, "InverseMovementDirection");
      
      if(inverted){
        minEncoderValue.encoderTicks = lowerlimit + 1000;
      }else{
        minEncoderValue.encoderTicks = upperlimit - 1000;
      }

      jLimits.setParameter(lowerlimit, upperlimit, true);
      joints[i].setConfigurationParameter(jLimits);
     // joints[i].setData(minEncoderValue);
    }

  // Bouml preserved body end 000A9C71
}

void YouBotManipulator::calibrateGripper(const bool forceCalibration) {
  // Bouml preserved body begin 000A9CF1
    // Calibrating Gripper
    bool doCalibration = true;
		configfile->readInto(doCalibration, "Gripper", "DoCalibration");
    if(useGripper && doCalibration){
      CalibrateGripper calibrate;
      calibrate.setParameter(forceCalibration);
      gripper->setConfigurationParameter(calibrate);
    }
  // Bouml preserved body end 000A9CF1
}

/// return the number of joints
int YouBotManipulator::getNumberJoints() {
	return numberArmJoints;
}

bool YouBotManipulator::hasGripper() {
	return isGripperInitialized;
}

///return a joint form the arm1
///@param armJointNumber 1-5 for the arm joints
YouBotJoint& YouBotManipulator::getArmJoint(const unsigned int armJointNumber) {
  // Bouml preserved body begin 0004F7F1

    if (armJointNumber <= 0 || armJointNumber > numberArmJoints) {
      throw std::out_of_range("Invalid Joint Number");
    }
    return joints[armJointNumber - 1];
  // Bouml preserved body end 0004F7F1
}

YouBotGripper& YouBotManipulator::getArmGripper() {
  // Bouml preserved body begin 0005F9F1
		if(!useGripper){
			throw std::runtime_error("The gripper is disabled!");
		}
    return *gripper;
  // Bouml preserved body end 0005F9F1
}

///commands positions or angles to all manipulator joints
///all positions will be set at the same time
///@param JointData the to command positions
void YouBotManipulator::setJointData(const std::vector<JointAngleSetpoint>& JointData) {
  // Bouml preserved body begin 0008FDF1
    if (JointData.size() != numberArmJoints)
      throw std::out_of_range("Wrong number of JointAngleSetpoints");

    ethercatMaster.AutomaticSendOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].setData(JointData[i]);
    ethercatMaster.AutomaticSendOn(true);

  // Bouml preserved body end 0008FDF1
}

///gets the position or angle of all manipulator joints which have been calculated from the actual encoder value
///These values are all read at the same time from the different joints 
///@param data returns the angles by reference
void YouBotManipulator::getJointData(std::vector<JointSensedAngle>& data) {
  // Bouml preserved body begin 0008FE71
    data.resize(numberArmJoints);
    ethercatMaster.AutomaticReceiveOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].getData(data[i]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FE71
}

///commands velocities to all manipulator joints
///all velocities will be set at the same time
///@param JointData the to command velocities
void YouBotManipulator::setJointData(const std::vector<JointVelocitySetpoint>& JointData) {
  // Bouml preserved body begin 0008FEF1
    if (JointData.size() != numberArmJoints)
      throw std::out_of_range("Wrong number of JointVelocitySetpoints");

    ethercatMaster.AutomaticSendOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].setData(JointData[i]);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 0008FEF1
}

///gets the velocities of all manipulator joints which have been calculated from the actual encoder values
///These values are all read at the same time from the different joints 
///@param data returns the velocities by reference
void YouBotManipulator::getJointData(std::vector<JointSensedVelocity>& data) {
  // Bouml preserved body begin 0008FF71
    data.resize(numberArmJoints);
    ethercatMaster.AutomaticReceiveOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].getData(data[i]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FF71
}

///commands current to all manipulator joints
///all current values will be set at the same time
///@param JointData the to command current
void YouBotManipulator::setJointData(const std::vector<JointCurrentSetpoint>& JointData) {
  // Bouml preserved body begin 000CDE71
    if (JointData.size() != numberArmJoints)
      throw std::out_of_range("Wrong number of JointCurrentSetpoint");

    ethercatMaster.AutomaticSendOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].setData(JointData[i]);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 000CDE71
}

///gets the motor currents of all manipulator joints which have been measured by a hal sensor
///These values are all read at the same time from the different joints 
///@param data returns the actual motor currents by reference
void YouBotManipulator::getJointData(std::vector<JointSensedCurrent>& data) {
  // Bouml preserved body begin 00090071
    data.resize(numberArmJoints);
    ethercatMaster.AutomaticReceiveOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].getData(data[i]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 00090071
}

///commands torque to all manipulator joints
///all torque values will be set at the same time
///@param JointData the to command torque 
void YouBotManipulator::setJointData(const std::vector<JointTorqueSetpoint>& JointData) {
  // Bouml preserved body begin 000CDEF1
    if (JointData.size() != numberArmJoints)
      throw std::out_of_range("Wrong number of JointTorqueSetpoint");

    ethercatMaster.AutomaticSendOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].setData(JointData[i]);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 000CDEF1
}

///gets the joint torque of all manipulator joints which have been calculated from the current
///These values are all read at the same time from the different joints 
///@param data returns the actual joint torque by reference
void YouBotManipulator::getJointData(std::vector<JointSensedTorque>& data) {
  // Bouml preserved body begin 000CDF71
    data.resize(numberArmJoints);
    ethercatMaster.AutomaticReceiveOn(false);
    for (unsigned int i = 0; i < numberArmJoints; i++)
        joints[i].getData(data[i]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 000CDF71
}

///does the commutation of the arm joints with firmware 2.0
void YouBotManipulator::commutationFirmware200() {
  // Bouml preserved body begin 0010D8F1
  
    InitializeJoint doInitialization;
    bool isInitialized = false;
    int noInitialization = 0;
    std::string jointName;
    unsigned int statusFlags;
    std::vector<bool> isCommutated;
    isCommutated.assign(numberArmJoints, false);
    unsigned int u = 0;
    JointCurrentSetpoint zerocurrent;
    zerocurrent.current = 0.0 * ampere;


    ClearMotorControllerTimeoutFlag clearTimeoutFlag;
    for (unsigned int i = 1; i <= numberArmJoints; i++) {
      this->getArmJoint(i).setConfigurationParameter(clearTimeoutFlag);
    }

    for (unsigned int i = 1; i <= numberArmJoints; i++) {
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

      JointRoundsPerMinuteSetpoint rpmSetpoint(100);
      
      ethercatMaster.AutomaticReceiveOn(false);
      for (unsigned int i = 1; i <= numberArmJoints; i++)
        this->getArmJoint(i).setData(rpmSetpoint);
      ethercatMaster.AutomaticReceiveOn(true);
     
      
      // check for the next 5 sec if the joints are commutated
      for (u = 1; u <= 5000; u++) {
        for (unsigned int i = 1; i <= numberArmJoints; i++) {
          this->getArmJoint(i).getStatus(statusFlags);
          if (statusFlags & INITIALIZED) {
            isCommutated[i - 1] = true;
            this->getArmJoint(i).setData(zerocurrent);
          }
        }
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
        if (isCommutated[0] && isCommutated[1] && isCommutated[2] && isCommutated[3] && isCommutated[4]) {
          break;
        }
        SLEEP_MILLISEC(1);
      }

      for (unsigned int i = 1; i <= numberArmJoints; i++) {
        this->getArmJoint(i).setData(zerocurrent);
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
        doInitialization.setParameter(false);
        this->getArmJoint(i).getConfigurationParameter(doInitialization);
        doInitialization.getParameter(isInitialized);
        if (!isInitialized) {
          std::stringstream jointNameStream;
          jointNameStream << "manipulator joint " << i;
          jointName = jointNameStream.str();
          throw std::runtime_error("Could not commutate " + jointName);
        }
      }
    }
  // Bouml preserved body end 0010D8F1
}

///does the commutation of the arm joints with firmware 1.48 and below
void YouBotManipulator::commutationFirmware148() {
  // Bouml preserved body begin 0010D971
  
    InitializeJoint doInitialization;
    bool isInitialized = false;
    int noInitialization = 0;
    std::string jointName;


    ClearMotorControllerTimeoutFlag clearTimeoutFlag;
    for (unsigned int i = 1; i <= numberArmJoints; i++) {
      this->getArmJoint(i).setConfigurationParameter(clearTimeoutFlag);
    }

    for (unsigned int i = 1; i <= numberArmJoints; i++) {
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

      ethercatMaster.AutomaticReceiveOn(false);
      for (unsigned int i = 1; i <= numberArmJoints; i++)
          this->getArmJoint(i).setConfigurationParameter(doInitialization);
      ethercatMaster.AutomaticReceiveOn(true);

      unsigned int statusFlags;
      std::vector<bool> isCommutated;
      isCommutated.assign(numberArmJoints, false);
      unsigned int u = 0;

      // check for the next 5 sec if the joints are commutated
      for (u = 1; u <= 5000; u++) {
        for (unsigned int i = 1; i <= numberArmJoints; i++) {
          if(!ethercatMaster.isThreadActive()){
            ethercatMaster.sendProcessData();
            ethercatMaster.receiveProcessData();
          }
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

      for (unsigned int i = 1; i <= numberArmJoints; i++) {
        doInitialization.setParameter(false);
        this->getArmJoint(i).getConfigurationParameter(doInitialization);
        doInitialization.getParameter(isInitialized);
        if (!isInitialized) {
          std::stringstream jointNameStream;
          jointNameStream << "manipulator joint " << i;
          jointName = jointNameStream.str();
          throw std::runtime_error("Could not commutate " + jointName);
        }
      }
    }


  // Bouml preserved body end 0010D971
}

void YouBotManipulator::initializeJoints() {
  // Bouml preserved body begin 00068071

 //   LOG(info) << "Initializing Joints";

    //enable overriding the number of joints
    if (configfile->keyExists("JointTopology", "NumberJoints"))
        configfile->readInto(numberArmJoints, "JointTopology", "NumberJoints");

    //get number of slaves
    unsigned int noSlaves = ethercatMaster.getNumberOfSlaves();

    if (noSlaves < numberArmJoints) {
      throw std::runtime_error("Not enough ethercat slaves were found to create a YouBotManipulator!");
    }

    unsigned int slaveNumber = 0;
    for (unsigned int i = 1; i <= numberArmJoints; i++) {
		std::stringstream jointConfigNameStream;
		jointConfigNameStream << "ManipulatorJoint" << i;
		std::string jointConfigName = jointConfigNameStream.str();

		configfile->readInto(slaveNumber, "JointTopology", jointConfigName);
		if (slaveNumber <= noSlaves) {
			joints.push_back(new YouBotJoint(slaveNumber));
		} else {
			throw std::out_of_range("The ethercat slave number is not available!");
		}
	}

    //Configure Joint Parameters
    std::string jointName;
    JointName jName;
    GearRatio gearRatio;
    EncoderTicksPerRound ticksPerRound;
    InverseMovementDirection inverseDir;
    double gearRatio_numerator = 0;
    double gearRatio_denominator = 1;
    FirmwareVersion firmwareTypeVersion;
    TorqueConstant torqueConst;
    double trajectory_p=0, trajectory_i=0, trajectory_d=0, trajectory_imax=0, trajectory_imin=0;


    for (unsigned int i = 0; i < numberArmJoints; i++) {
      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();


      joints[i].getConfigurationParameter(firmwareTypeVersion);
      std::string version;
      int controllerType;
      std::string firmwareVersion;
      firmwareTypeVersion.getParameter(controllerType, firmwareVersion);

      string name;
      configfile->readInto(name, jointName, "JointName");
      jName.setParameter(name);

      LOG(info) << name << "\t Controller Type: " << controllerType << "  Firmware version: " << firmwareVersion;

      if (this->controllerType != controllerType && alternativeControllerType != controllerType) {
        std::stringstream ss;
        ss << "The youBot manipulator motor controller have to be of type: " << this->controllerType << " or " << alternativeControllerType;
        throw std::runtime_error(ss.str().c_str());
      }

      //check if firmware is supported
      bool isfirmwareSupported = false;
      for(unsigned int d = 0; d < supportedFirmwareVersions.size(); d++){
        if(this->supportedFirmwareVersions[d] == firmwareVersion){
          isfirmwareSupported = true;
          break;
        }
      }
      
      if(!isfirmwareSupported){
        throw std::runtime_error("Unsupported firmware version: " + firmwareVersion);
      }
      
      if(this->actualFirmwareVersionAllJoints == ""){
        this->actualFirmwareVersionAllJoints = firmwareVersion;
      }
      
      if(!(firmwareVersion == this->actualFirmwareVersionAllJoints)){
         throw std::runtime_error("All joints must have the same firmware version!");
      }

      configfile->readInto(gearRatio_numerator, jointName, "GearRatio_numerator");
      configfile->readInto(gearRatio_denominator, jointName, "GearRatio_denominator");
      gearRatio.setParameter(gearRatio_numerator / gearRatio_denominator);
      int ticks;
      configfile->readInto(ticks, jointName, "EncoderTicksPerRound");
      ticksPerRound.setParameter(ticks);
      
      double torqueConstant;
      configfile->readInto(torqueConstant, jointName, "TorqueConstant_[newton_meter_divided_by_ampere]");
      torqueConst.setParameter(torqueConstant);
      
      bool invdir = false;
      configfile->readInto(invdir, jointName, "InverseMovementDirection");
      inverseDir.setParameter(invdir);

      joints[i].setConfigurationParameter(jName);
      joints[i].setConfigurationParameter(gearRatio);
      joints[i].setConfigurationParameter(ticksPerRound);
      joints[i].setConfigurationParameter(torqueConst);
      joints[i].setConfigurationParameter(inverseDir);
      
      //Joint Trajectory Controller
      if(ethercatMaster.isThreadActive()){
				configfile->readInto(trajectory_p, jointName, "trajectory_controller_P");
        configfile->readInto(trajectory_i, jointName, "trajectory_controller_I");
        configfile->readInto(trajectory_d, jointName, "trajectory_controller_D");
        configfile->readInto(trajectory_imax, jointName, "trajectory_controller_I_max");
        configfile->readInto(trajectory_imin, jointName, "trajectory_controller_I_min");
        joints[i].trajectoryController.setConfigurationParameter(trajectory_p, trajectory_i, trajectory_d, trajectory_imax, trajectory_imin);
				joints[i].trajectoryController.setEncoderTicksPerRound(ticks);
        joints[i].trajectoryController.setGearRatio(gearRatio_numerator / gearRatio_denominator);
        joints[i].trajectoryController.setInverseMovementDirection(invdir);
        ethercatMasterWithThread->registerJointTrajectoryController(&(joints[i].trajectoryController), joints[i].getJointNumber());
			}
    }


    configfile->readInto(useGripper, "Gripper", "EnableGripper");

    if (useGripper) {
        try {
            //Initializing Gripper
            configfile->readInto(slaveNumber, "JointTopology", "ManipulatorJoint5");
            this->gripper.reset(new YouBotGripper(slaveNumber));
            BarSpacingOffset barOffest;
            MaxTravelDistance maxDistance;
            MaxEncoderValue maxEncoder;
            GripperBarName BarName;
            double dummy = 0;
            int controllerType;
            double firmwareVersion;
            string barname;

            GripperFirmwareVersion gripperVersion;
            this->gripper->getConfigurationParameter(gripperVersion);
            gripperVersion.getParameter(controllerType, firmwareVersion);

            LOG(info) << "Gripper" << "\t\t Controller Type: " << controllerType << "  Firmware version: " << firmwareVersion;

            // Gripper Bar 1
            configfile->readInto(barname, "GripperBar1", "BarName");
            BarName.setParameter(barname);
            this->gripper->getGripperBar1().setConfigurationParameter(BarName);

            configfile->readInto(dummy, "GripperBar1", "BarSpacingOffset_[meter]");
            barOffest.setParameter(dummy * meter);
            this->gripper->getGripperBar1().setConfigurationParameter(barOffest);

            configfile->readInto(dummy, "GripperBar1", "MaxTravelDistance_[meter]");
            maxDistance.setParameter(dummy * meter);
            this->gripper->getGripperBar1().setConfigurationParameter(maxDistance);

            int maxenc = 0;
            configfile->readInto(maxenc, "GripperBar1", "MaxEncoderValue");
            maxEncoder.setParameter(maxenc);
            this->gripper->getGripperBar1().setConfigurationParameter(maxEncoder);

            int stallThreshold = 0;
            configfile->readInto(stallThreshold, "GripperBar1", "StallGuard2Threshold");
            StallGuard2Threshold threshold;
            threshold.setParameter(stallThreshold);
            this->gripper->getGripperBar1().setConfigurationParameter(threshold);

            bool stallGuardFilter = false;
            configfile->readInto(stallGuardFilter, "GripperBar1", "StallGuard2FilterEnable");
            StallGuard2FilterEnable filter;
            filter.setParameter(stallGuardFilter);
            this->gripper->getGripperBar1().setConfigurationParameter(filter);

            // Gripper Bar 2
            configfile->readInto(barname, "GripperBar2", "BarName");
            BarName.setParameter(barname);
            this->gripper->getGripperBar2().setConfigurationParameter(BarName);

            configfile->readInto(dummy, "GripperBar2", "BarSpacingOffset_[meter]");
            barOffest.setParameter(dummy * meter);
            this->gripper->getGripperBar2().setConfigurationParameter(barOffest);

            configfile->readInto(dummy, "GripperBar2", "MaxTravelDistance_[meter]");
            maxDistance.setParameter(dummy * meter);
            this->gripper->getGripperBar2().setConfigurationParameter(maxDistance);

            configfile->readInto(maxenc, "GripperBar2", "MaxEncoderValue");
            maxEncoder.setParameter(maxenc);
            this->gripper->getGripperBar2().setConfigurationParameter(maxEncoder);

            configfile->readInto(stallThreshold, "GripperBar2", "StallGuard2Threshold");
            threshold.setParameter(stallThreshold);
            this->gripper->getGripperBar2().setConfigurationParameter(threshold);

            configfile->readInto(stallGuardFilter, "GripperBar2", "StallGuard2FilterEnable");
            filter.setParameter(stallGuardFilter);
            this->gripper->getGripperBar2().setConfigurationParameter(filter);
        } catch (std::exception &e) {
            isGripperInitialized = false;
            return;
        }

        isGripperInitialized = true;
    }

    return;
  // Bouml preserved body end 00068071
}


} // namespace youbot
