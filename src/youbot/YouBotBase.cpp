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
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"
namespace youbot {

YouBotBase::YouBotBase(const std::string name, const std::string configFilePath)
: ethercatMaster(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath)) {
  // Bouml preserved body begin 00067E71

    this->controllerType = 174;
    this->alternativeControllerType = 1632;
    this->supportedFirmwareVersions.push_back("148");
    this->supportedFirmwareVersions.push_back("200");
    this->actualFirmwareVersionAllJoints = "";

    string filename;
    filename = name;
    filename.append(".cfg");

    configfile.reset(new ConfigFile(filename, configFilePath));
    
    if(ethercatMaster.isThreadActive()){
			ethercatMasterWithThread = static_cast<EthercatMasterWithThread*>(&(EthercatMaster::getInstance()));
		}else{
			ethercatMasterWithThread = NULL;
		}
    
    this->initializeJoints();

    this->initializeKinematic();

  // Bouml preserved body end 00067E71
}

YouBotBase::~YouBotBase() {
  // Bouml preserved body begin 00067EF1
  if(ethercatMaster.isThreadActive()){
			for (unsigned int i = 0; i < BASEJOINTS; i++) {
				ethercatMasterWithThread->deleteJointTrajectoryControllerRegistration(this->joints[i].getJointNumber());
			}
		}
  // Bouml preserved body end 00067EF1
}

///does the sine commutation of the base joints
void YouBotBase::doJointCommutation() {
  // Bouml preserved body begin 0008A9F1

  if(this->actualFirmwareVersionAllJoints == "148"){
    this->commutationFirmware148();
  }else if(this->actualFirmwareVersionAllJoints == "200" ){
    this->commutationFirmware200();
  }else{
    throw std::runtime_error("Unable to commutate joints - Unsupported firmware version!");
  }
  // Bouml preserved body end 0008A9F1
}

///return a joint form the base
///@param baseJointNumber 1-4 for the base joints
YouBotJoint& YouBotBase::getBaseJoint(const unsigned int baseJointNumber) {
  // Bouml preserved body begin 0004F771
    if (baseJointNumber <= 0 || baseJointNumber > BASEJOINTS) {
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
    wheelPositions.assign(BASEJOINTS, dummy);

    ethercatMaster.AutomaticReceiveOn(false);
    joints[0].getData(sensedPos);
    wheelPositions[0] = sensedPos.angle;
    joints[1].getData(sensedPos);
    wheelPositions[1] = sensedPos.angle;
    joints[2].getData(sensedPos);
    wheelPositions[2] = sensedPos.angle;
    joints[3].getData(sensedPos);
    wheelPositions[3] = sensedPos.angle;
    ethercatMaster.AutomaticReceiveOn(true);

    youBotBaseKinematic.wheelPositionsToCartesianPosition(wheelPositions, longitudinalPosition, transversalPosition, orientation);
  // Bouml preserved body end 000514F1
}

///sets the cartesien base position
///@param longitudinalPosition is the forward or backward position
///@param transversalPosition is the sideway position
///@param orientation is the rotation around the center of the YouBot
void YouBotBase::setBasePosition(const quantity<si::length>& longitudinalPosition, const quantity<si::length>& transversalPosition, const quantity<plane_angle>& orientation) {
  // Bouml preserved body begin 000C0971

    std::vector<quantity<plane_angle> > wheelPositions;
    quantity<plane_angle> dummy;
    JointAngleSetpoint setpointPos;
    wheelPositions.assign(BASEJOINTS, dummy);
    JointSensedAngle sensedPos;

    youBotBaseKinematic.cartesianPositionToWheelPositions(longitudinalPosition, transversalPosition, orientation, wheelPositions);
    
    if (wheelPositions.size() < BASEJOINTS)
      throw std::out_of_range("To less wheel velocities");
    
    joints[0].setEncoderToZero();
    joints[1].setEncoderToZero();
    joints[2].setEncoderToZero();
    joints[3].setEncoderToZero();
    SLEEP_MILLISEC(10);

    ethercatMaster.AutomaticSendOn(false);
    joints[0].getData(sensedPos);
    setpointPos.angle = sensedPos.angle + wheelPositions[0];
    joints[0].setData(setpointPos);
    
    joints[1].getData(sensedPos);
    setpointPos.angle = sensedPos.angle + wheelPositions[1];
    joints[1].setData(setpointPos);
    
    joints[2].getData(sensedPos);
    setpointPos.angle = sensedPos.angle + wheelPositions[2];
    joints[2].setData(setpointPos);
    
    joints[3].getData(sensedPos);
    setpointPos.angle = sensedPos.angle + wheelPositions[3];
    joints[3].setData(setpointPos);
    ethercatMaster.AutomaticSendOn(true);

  // Bouml preserved body end 000C0971
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
    wheelVelocities.assign(BASEJOINTS, dummy);

    ethercatMaster.AutomaticReceiveOn(false);
    joints[0].getData(sensedVel);
    wheelVelocities[0] = sensedVel.angularVelocity;
    joints[1].getData(sensedVel);
    wheelVelocities[1] = sensedVel.angularVelocity;
    joints[2].getData(sensedVel);
    wheelVelocities[2] = sensedVel.angularVelocity;
    joints[3].getData(sensedVel);
    wheelVelocities[3] = sensedVel.angularVelocity;
    ethercatMaster.AutomaticReceiveOn(true);

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

    if (wheelVelocities.size() < BASEJOINTS)
      throw std::out_of_range("To less wheel velocities");

    ethercatMaster.AutomaticSendOn(false);
    setVel.angularVelocity = wheelVelocities[0];
    joints[0].setData(setVel);
    setVel.angularVelocity = wheelVelocities[1];
    joints[1].setData(setVel);
    setVel.angularVelocity = wheelVelocities[2];
    joints[2].setData(setVel);
    setVel.angularVelocity = wheelVelocities[3];
    joints[3].setData(setVel);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 0004DD71
}

///commands positions or angles to all base joints
///all positions will be set at the same time
///@param JointData the to command positions
void YouBotBase::setJointData(const std::vector<JointAngleSetpoint>& JointData) {
  // Bouml preserved body begin 0008F9F1
    if (JointData.size() != BASEJOINTS)
      throw std::out_of_range("Wrong number of JointAngleSetpoints");

    ethercatMaster.AutomaticSendOn(false);
    joints[0].setData(JointData[0]);
    joints[1].setData(JointData[1]);
    joints[2].setData(JointData[2]);
    joints[3].setData(JointData[3]);
    ethercatMaster.AutomaticSendOn(true);

  // Bouml preserved body end 0008F9F1
}

///gets the position or angle of all base joints which have been calculated from the actual encoder value
///These values are all read at the same time from the different joints 
///@param data returns the angles by reference
void YouBotBase::getJointData(std::vector<JointSensedAngle>& data) {
  // Bouml preserved body begin 0008FBF1
    data.resize(BASEJOINTS);
    ethercatMaster.AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FBF1
}

///commands velocities to all base joints
///all velocities will be set at the same time
///@param JointData the to command velocities
void YouBotBase::setJointData(const std::vector<JointVelocitySetpoint>& JointData) {
  // Bouml preserved body begin 0008FA71
    if (JointData.size() != BASEJOINTS)
      throw std::out_of_range("Wrong number of JointVelocitySetpoints");

    ethercatMaster.AutomaticSendOn(false);
    joints[0].setData(JointData[0]);
    joints[1].setData(JointData[1]);
    joints[2].setData(JointData[2]);
    joints[3].setData(JointData[3]);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 0008FA71
}

///gets the velocities of all base joints which have been calculated from the actual encoder values
///These values are all read at the same time from the different joints 
///@param data returns the velocities by reference
void YouBotBase::getJointData(std::vector<JointSensedVelocity>& data) {
  // Bouml preserved body begin 0008FC71
    data.resize(BASEJOINTS);
    ethercatMaster.AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FC71
}

///commands current to all base joints
///all current values will be set at the same time
///@param JointData the to command current
void YouBotBase::setJointData(const std::vector<JointCurrentSetpoint>& JointData) {
  // Bouml preserved body begin 000CDFF1
    if (JointData.size() != BASEJOINTS)
      throw std::out_of_range("Wrong number of JointCurrentSetpoint");

    ethercatMaster.AutomaticSendOn(false);
    joints[0].setData(JointData[0]);
    joints[1].setData(JointData[1]);
    joints[2].setData(JointData[2]);
    joints[3].setData(JointData[3]);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 000CDFF1
}

///gets the motor currents of all base joints which have been measured by a hal sensor
///These values are all read at the same time from the different joints 
///@param data returns the actual motor currents by reference
void YouBotBase::getJointData(std::vector<JointSensedCurrent>& data) {
  // Bouml preserved body begin 0008FD71
    data.resize(BASEJOINTS);
    ethercatMaster.AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 0008FD71
}

///commands torque to all base joints
///all torque values will be set at the same time
///@param JointData the to command torque 
void YouBotBase::setJointData(const std::vector<JointTorqueSetpoint>& JointData) {
  // Bouml preserved body begin 000CE071
    if (JointData.size() != BASEJOINTS)
      throw std::out_of_range("Wrong number of JointTorqueSetpoint");

    ethercatMaster.AutomaticSendOn(false);
    joints[0].setData(JointData[0]);
    joints[1].setData(JointData[1]);
    joints[2].setData(JointData[2]);
    joints[3].setData(JointData[3]);
    ethercatMaster.AutomaticSendOn(true);
  // Bouml preserved body end 000CE071
}

///gets the joint torque of all base joints which have been calculated from the current
///These values are all read at the same time from the different joints 
///@param data returns the actual joint torque by reference
void YouBotBase::getJointData(std::vector<JointSensedTorque>& data) {
  // Bouml preserved body begin 000CE0F1
    data.resize(BASEJOINTS);
    ethercatMaster.AutomaticReceiveOn(false);
    joints[0].getData(data[0]);
    joints[1].getData(data[1]);
    joints[2].getData(data[2]);
    joints[3].getData(data[3]);
    ethercatMaster.AutomaticReceiveOn(true);
  // Bouml preserved body end 000CE0F1
}

///does the commutation of the arm joints with firmware 2.0
void YouBotBase::commutationFirmware200() {
  // Bouml preserved body begin 0010D9F1
  
    InitializeJoint doInitialization;
    bool isInitialized = false;
    int noInitialization = 0;
    std::string jointName;
    unsigned int statusFlags;
    std::vector<bool> isCommutated;
    isCommutated.assign(BASEJOINTS, false);
    unsigned int u = 0;
    JointCurrentSetpoint zerocurrent;
    zerocurrent.current = 0.0 * ampere;


    ClearMotorControllerTimeoutFlag clearTimeoutFlag;
    for (unsigned int i = 1; i <= BASEJOINTS; i++) {
      this->getBaseJoint(i).setConfigurationParameter(clearTimeoutFlag);
    }

    for (unsigned int i = 1; i <= BASEJOINTS; i++) {
      doInitialization.setParameter(false);
      this->getBaseJoint(i).getConfigurationParameter(doInitialization);
      doInitialization.getParameter(isInitialized);
      if (!isInitialized) {
        noInitialization++;
      }
    }

    if (noInitialization != 0) {
      LOG(info) << "Base Joint Commutation with firmware 2.0";
      doInitialization.setParameter(true);

      JointRoundsPerMinuteSetpoint rpmSetpoint(100);
	      
      ethercatMaster.AutomaticReceiveOn(false);
      this->getBaseJoint(1).setData(rpmSetpoint);
      this->getBaseJoint(2).setData(rpmSetpoint);
      this->getBaseJoint(3).setData(rpmSetpoint);
      this->getBaseJoint(4).setData(rpmSetpoint);
      ethercatMaster.AutomaticReceiveOn(true);
      rpmSetpoint.rpm = 0;
     
      
      // check for the next 5 sec if the joints are commutated
      for (u = 1; u <= 5000; u++) {
        for (unsigned int i = 1; i <= BASEJOINTS; i++) {
          this->getBaseJoint(i).getStatus(statusFlags);
          if (statusFlags & INITIALIZED) {
            isCommutated[i - 1] = true;
            this->getBaseJoint(i).setData(rpmSetpoint);
          }
        }
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
        if (isCommutated[0] && isCommutated[1] && isCommutated[2] && isCommutated[3]) {
          break;
        }
        SLEEP_MILLISEC(1);
      }

      for (unsigned int i = 1; i <= BASEJOINTS; i++) {
        this->getBaseJoint(i).setData(rpmSetpoint);
        if(!ethercatMaster.isThreadActive()){
          ethercatMaster.sendProcessData();
          ethercatMaster.receiveProcessData();
        }
        doInitialization.setParameter(false);
        this->getBaseJoint(i).getConfigurationParameter(doInitialization);
        doInitialization.getParameter(isInitialized);
        if (!isInitialized) {
          std::stringstream jointNameStream;
          jointNameStream << "base joint " << i;
          jointName = jointNameStream.str();
          throw std::runtime_error("Could not commutate " + jointName);
        }
      }
    }
  // Bouml preserved body end 0010D9F1
}

///does the commutation of the arm joints with firmware 1.48 and below
void YouBotBase::commutationFirmware148() {
  // Bouml preserved body begin 0010DA71
  
    InitializeJoint doInitialization;
    bool isInitialized = false;
    int noInitialization = 0;
    std::string jointName;


    ClearMotorControllerTimeoutFlag clearTimeoutFlag;
    for (unsigned int i = 1; i <= BASEJOINTS; i++) {
      this->getBaseJoint(i).setConfigurationParameter(clearTimeoutFlag);
    }

    for (unsigned int i = 1; i <= BASEJOINTS; i++) {
      doInitialization.setParameter(false);
      this->getBaseJoint(i).getConfigurationParameter(doInitialization);
      doInitialization.getParameter(isInitialized);
      if (!isInitialized) {
        noInitialization++;
      }
    }

    if (noInitialization != 0) {
      LOG(info) << "Base Joint Commutation with firmware 1.48";
      doInitialization.setParameter(true);

      ethercatMaster.AutomaticReceiveOn(false);
      this->getBaseJoint(1).setConfigurationParameter(doInitialization);
      this->getBaseJoint(2).setConfigurationParameter(doInitialization);
      this->getBaseJoint(3).setConfigurationParameter(doInitialization);
      this->getBaseJoint(4).setConfigurationParameter(doInitialization);
      ethercatMaster.AutomaticReceiveOn(true);

      unsigned int statusFlags;
      std::vector<bool> isCommutated;
      isCommutated.assign(BASEJOINTS, false);
      unsigned int u = 0;

      // check for the next 5 sec if the joints are commutated
      for (u = 1; u <= 5000; u++) {
        for (unsigned int i = 1; i <= BASEJOINTS; i++) {
          if(!ethercatMaster.isThreadActive()){
            ethercatMaster.sendProcessData();
            ethercatMaster.receiveProcessData();
          }
          this->getBaseJoint(i).getStatus(statusFlags);
          if (statusFlags & INITIALIZED) {
            isCommutated[i - 1] = true;
          }
        }
        if (isCommutated[0] && isCommutated[1] && isCommutated[2] && isCommutated[3]) {
          break;
        }
        SLEEP_MILLISEC(1);
      }

      SLEEP_MILLISEC(10); // the controller likes it

      for (unsigned int i = 1; i <= BASEJOINTS; i++) {
        doInitialization.setParameter(false);
        this->getBaseJoint(i).getConfigurationParameter(doInitialization);
        doInitialization.getParameter(isInitialized);
        if (!isInitialized) {
          std::stringstream jointNameStream;
          jointNameStream << "base joint " << i;
          jointName = jointNameStream.str();
          throw std::runtime_error("Could not commutate " + jointName);
        }
      }
    }


  // Bouml preserved body end 0010DA71
}

void YouBotBase::initializeJoints() {
  // Bouml preserved body begin 000464F1

    LOG(trace) << "Initializing Joints";

    //get number of slaves
    unsigned int noSlaves = ethercatMaster.getNumberOfSlaves();

    if (noSlaves < BASEJOINTS) {
      throw std::out_of_range("Not enough ethercat slaves were found to create a YouBotBase!");
    }

    //read Joint Topology from config file
    //  configfile.setSection("JointTopology");

    //check if enough slave exist to create YouBotJoint for the slave numbers from config file
    //if enough slave exist create YouBotJoint and store it in the joints vector
    unsigned int slaveNumber = 0;
    configfile->readInto(slaveNumber, "JointTopology", "BaseLeftFront");
    if (slaveNumber <= noSlaves) {
      joints.push_back(new YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "BaseRightFront");
    if (slaveNumber <= noSlaves) {
      joints.push_back(new YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "BaseLeftBack");
    if (slaveNumber <= noSlaves) {
      joints.push_back(new YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }

    configfile->readInto(slaveNumber, "JointTopology", "BaseRightBack");
    if (slaveNumber <= noSlaves) {
      joints.push_back(new YouBotJoint(slaveNumber));
    } else {
      throw std::out_of_range("The ethercat slave number is not available!");
    }


    //Configure Joint Parameters
    std::string jointName;
    JointName jName;
    GearRatio gearRatio;
    EncoderTicksPerRound ticksPerRound;
    InverseMovementDirection inverseDir;
    FirmwareVersion firmwareTypeVersion;
    TorqueConstant torqueConst;

    double trajectory_p=0, trajectory_i=0, trajectory_d=0, trajectory_imax=0, trajectory_imin=0;
    double gearRatio_numerator = 0;
    double gearRatio_denominator = 1;
    JointLimits jLimits;

    for (unsigned int i = 0; i < BASEJOINTS; i++) {
      std::stringstream jointNameStream;
      jointNameStream << "Joint_" << i + 1;
      jointName = jointNameStream.str();
      //  configfile.setSection(jointName.c_str());

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
        ss << "The youBot base motor controller have to be of type: " << this->controllerType << " or " << alternativeControllerType;
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
        throw std::runtime_error("Unsupported firmware version: "+ firmwareVersion);
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
   
      long upperlimit = 0, lowerlimit = 0;
      configfile->readInto(lowerlimit, jointName, "LowerLimit_[encoderTicks]");
      configfile->readInto(upperlimit, jointName, "UpperLimit_[encoderTicks]");

      jLimits.setParameter(lowerlimit, upperlimit, false);
      joints[i].setConfigurationParameter(jLimits);
      
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

    return;
  // Bouml preserved body end 000464F1
}

void YouBotBase::initializeKinematic() {
  // Bouml preserved body begin 0004DDF1
    FourSwedishWheelOmniBaseKinematicConfiguration kinematicConfig;

    //read the kinematics parameter from a config file
    configfile->readInto(kinematicConfig.rotationRatio, "YouBotKinematic", "RotationRatio");
    configfile->readInto(kinematicConfig.slideRatio, "YouBotKinematic", "SlideRatio");
    double dummy = 0;
    configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontAndRearWheels_[meter]");
    kinematicConfig.lengthBetweenFrontAndRearWheels = dummy * meter;
    configfile->readInto(dummy, "YouBotKinematic", "LengthBetweenFrontWheels_[meter]");
    kinematicConfig.lengthBetweenFrontWheels = dummy * meter;
    configfile->readInto(dummy, "YouBotKinematic", "WheelRadius_[meter]");
    kinematicConfig.wheelRadius = dummy * meter;


    youBotBaseKinematic.setConfiguration(kinematicConfig);
  // Bouml preserved body end 0004DDF1
}


} // namespace youbot
