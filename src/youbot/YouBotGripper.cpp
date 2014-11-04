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
#include "youbot_driver/youbot/YouBotGripper.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"
namespace youbot {

YouBotGripper::YouBotGripper(const unsigned int jointNo, const std::string& configFilePath) {
  // Bouml preserved body begin 0005EFF1
    this->jointNumber = jointNo;
    this->mailboxMsgRetries = 200;
    this->timeTillNextMailboxUpdate = 1; //ms

    ethercatMaster = &(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath));
    bar1.reset(new YouBotGripperBar(0, jointNo, configFilePath));
    bar2.reset(new YouBotGripperBar(1, jointNo, configFilePath));

  // Bouml preserved body end 0005EFF1
}

YouBotGripper::~YouBotGripper() {
  // Bouml preserved body begin 0005F071
  // Bouml preserved body end 0005F071
}

void YouBotGripper::getConfigurationParameter(GripperParameter& parameter) const {
  // Bouml preserved body begin 0005FBF1
    throw std::runtime_error("Please use YouBotGripperParameter");
  // Bouml preserved body end 0005FBF1
}

void YouBotGripper::setConfigurationParameter(const GripperParameter& parameter) {
  // Bouml preserved body begin 0005FA71
    throw std::runtime_error("Please use YouBotGripperParameter");
  // Bouml preserved body end 0005FA71
}

void YouBotGripper::getConfigurationParameter(GripperFirmwareVersion& parameter) const {
  // Bouml preserved body begin 000BEF71

    YouBotSlaveMailboxMsg message;
    parameter.getYouBotMailboxMsg(message);

    bool unvalid = true;
    unsigned int retry = 0;

    ethercatMaster->setMailboxMsgBuffer(message, this->jointNumber);
    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
      if (ethercatMaster->getMailboxMsgBuffer(message, this->jointNumber)) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
    } while (retry < mailboxMsgRetries && unvalid);

    if (unvalid) {
      this->parseMailboxStatusFlags(message);
      throw std::runtime_error("Unable to get firmware version of the gripper");
      return;
    }

    char versionString[8] = {0};
    //  versionString[0] = message.stctInput.replyAddress;
    versionString[0] = message.stctInput.moduleAddress;
    versionString[1] = message.stctInput.status;
    versionString[2] = message.stctInput.commandNumber;
    versionString[3] = message.stctInput.value >> 24;
    versionString[4] = message.stctInput.value >> 16;
    versionString[5] = message.stctInput.value >> 8;
    versionString[6] = message.stctInput.value & 0xff;

    // LOG(warning) <<"version: "<< versionString;
    int controllerType = 0;
    float firmwareVersion = 0;
    sscanf(versionString, "%dV%f", &controllerType, &firmwareVersion); //KR842V20


    parameter.setParameter(controllerType, firmwareVersion);

    return;
  // Bouml preserved body end 000BEF71
}

void YouBotGripper::setConfigurationParameter(const CalibrateGripper& parameter) {
  // Bouml preserved body begin 00048271

		char index = 16; // Parameter 0 to 15 of bank 2 are password protected
		YouBotSlaveMailboxMsg IsCalibratedReadMessage;
		IsCalibratedReadMessage.stctOutput.moduleAddress = GRIPPER;
		IsCalibratedReadMessage.stctOutput.commandNumber = GGP;
		IsCalibratedReadMessage.stctOutput.typeNumber = index;
		IsCalibratedReadMessage.stctOutput.motorNumber = USER_VARIABLE_BANK;
		IsCalibratedReadMessage.stctOutput.value = 0;
		IsCalibratedReadMessage.stctInput.value = 0;

		YouBotSlaveMailboxMsg IsCalibratedSetMessage;
		IsCalibratedSetMessage.stctOutput.moduleAddress = GRIPPER;
		IsCalibratedSetMessage.stctOutput.commandNumber = SGP;
		IsCalibratedSetMessage.stctOutput.typeNumber = index;
		IsCalibratedSetMessage.stctOutput.motorNumber = USER_VARIABLE_BANK;
		IsCalibratedSetMessage.stctOutput.value = 1;
		

		bool doCalibration = true;
		if (parameter.value == false) {
			if (!retrieveValueFromMotorContoller(IsCalibratedReadMessage)) {
				IsCalibratedReadMessage.stctInput.value = 0;
			}
			
      if (IsCalibratedReadMessage.stctInput.value == 1) {
        doCalibration = false;
      }
		}
			
		if(doCalibration){
      LOG(info) << "Calibrate Gripper";
 
      YouBotSlaveMailboxMsg message;

      unsigned int maxenc = 0;
      MaxEncoderValue maxencoder;
      bar1->getConfigurationParameter(maxencoder);
      maxencoder.getParameter(maxenc);

      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.commandNumber = MVP;
      message.stctOutput.typeNumber = 1; //move gripper relative
      message.stctOutput.value = -maxenc;
      message.stctOutput.motorNumber = 0; //move bar 0
      setValueToMotorContoller(message);
      message.stctOutput.motorNumber = 1; //move bar 1
      setValueToMotorContoller(message);

      //open gripper
      TargetPositionReached bar1TargetReched;
      TargetPositionReached bar2TargetReched;
      bool targetReachedBar1 = false;
      bool targetReachedBar2 = false;

      for (int i = 0; i < 40; i++) {
        SLEEP_MILLISEC(100);
        bar1->getConfigurationParameter(bar1TargetReched);
        bar1TargetReched.getParameter(targetReachedBar1);
        bar2->getConfigurationParameter(bar2TargetReched);
        bar2TargetReched.getParameter(targetReachedBar2);
        if (targetReachedBar1 && targetReachedBar2){
          break;
        }
      }

      //close gripper
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.commandNumber = MVP;
      message.stctOutput.typeNumber = 1; //move gripper relative
      message.stctOutput.value = maxenc;
      message.stctOutput.motorNumber = 0; //move bar 0
      setValueToMotorContoller(message);
      message.stctOutput.motorNumber = 1; //move bar 1
      setValueToMotorContoller(message);
      
      targetReachedBar1 = false;
      targetReachedBar2 = false;

      for (int i = 0; i < 40; i++) {
        SLEEP_MILLISEC(100);
        bar1->getConfigurationParameter(bar1TargetReched);
        bar1TargetReched.getParameter(targetReachedBar1);
        bar2->getConfigurationParameter(bar2TargetReched);
        bar2TargetReched.getParameter(targetReachedBar2);
        if (targetReachedBar1 && targetReachedBar2)
          break;
      }
      
      //stop Gripper motor
      /*
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.commandNumber = MST;
      message.stctOutput.value = 0;
      message.stctOutput.motorNumber = 0; //move bar 0
      setValueToMotorContoller(message);
      message.stctOutput.motorNumber = 1; //move bar 1
      setValueToMotorContoller(message);
*/
      
      // set pose to zero as reference
      ActualPosition actualPose;
      actualPose.setParameter(0);
      bar1->setConfigurationParameter(actualPose);
      bar2->setConfigurationParameter(actualPose);
			
			 // set a flag in the user variable to remember that it is calibrated
      this->setValueToMotorContoller(IsCalibratedSetMessage);
		}

  // Bouml preserved body end 00048271
}

void YouBotGripper::getConfigurationParameter(YouBotSlaveMailboxMsg& parameter) const {
  // Bouml preserved body begin 000DE971
    if (!retrieveValueFromMotorContoller(parameter)) {
      throw JointParameterException("Unable to get parameter from the gripper");
    }
    this->parseMailboxStatusFlags(parameter);
  // Bouml preserved body end 000DE971
}

void YouBotGripper::getData(const GripperData& data) const {
  // Bouml preserved body begin 0005FB71
    LOG(info) << "Nothing to do";
  // Bouml preserved body end 0005FB71
}

void YouBotGripper::setData(const GripperData& data) {
  // Bouml preserved body begin 0005FAF1
    LOG(info) << "Nothing to do";
  // Bouml preserved body end 0005FAF1
}

void YouBotGripper::getData(OneDOFGripperData& data) const {
  // Bouml preserved body begin 000483F1
    LOG(info) << "Nothing to do";
  // Bouml preserved body end 000483F1
}

void YouBotGripper::setData(const OneDOFGripperData& data) {
  // Bouml preserved body begin 00048371
    LOG(info) << "Nothing to do";
  // Bouml preserved body end 00048371
}

void YouBotGripper::setData(const GripperBarSpacingSetPoint& barSpacing) {
  // Bouml preserved body begin 0005F8F1
    
    GripperBarPositionSetPoint setpointBar1;
    GripperBarPositionSetPoint setpointBar2;
		
		setpointBar1.barPosition = barSpacing.barSpacing/2.0;
		setpointBar2.barPosition = barSpacing.barSpacing/2.0;

    bar1->setData(setpointBar1);
    bar2->setData(setpointBar2);

  // Bouml preserved body end 0005F8F1
}

void YouBotGripper::getData(GripperSensedBarSpacing& barSpacing) const {
  // Bouml preserved body begin 0005F971
		GripperSensedBarPosition bar1Position;
		GripperSensedBarPosition bar2Position;
		bar1->getData(bar1Position);
		bar2->getData(bar2Position);
		
    barSpacing.barSpacing = bar1Position.barPosition + bar2Position.barPosition;

  // Bouml preserved body end 0005F971
}

void YouBotGripper::open() {
  // Bouml preserved body begin 000E3BF1
   
    MaxEncoderValue maxEnc;
    unsigned int bar1MaxEncoderValue = 0;
    unsigned int bar2MaxEncoderValue = 0;
  
    bar1->getConfigurationParameter(maxEnc);
    maxEnc.getParameter(bar1MaxEncoderValue);
    bar2->getConfigurationParameter(maxEnc);
    maxEnc.getParameter(bar2MaxEncoderValue);
    
    GripperBarEncoderSetpoint setpointBar1;
    GripperBarEncoderSetpoint setpointBar2;
    setpointBar1.barEncoder = bar1MaxEncoderValue;
    setpointBar2.barEncoder = bar2MaxEncoderValue;

    bar1->setData(setpointBar1);
    bar2->setData(setpointBar2);

  // Bouml preserved body end 000E3BF1
}

void YouBotGripper::close() {
  // Bouml preserved body begin 00103D71
    GripperBarEncoderSetpoint setpointBar1;
    GripperBarEncoderSetpoint setpointBar2;
    setpointBar1.barEncoder = 0;
    setpointBar2.barEncoder = 0;

    bar1->setData(setpointBar1);
    bar2->setData(setpointBar2);
  // Bouml preserved body end 00103D71
}

YouBotGripperBar& YouBotGripper::getGripperBar1() {
  // Bouml preserved body begin 000E0FF1
    if (bar1 == NULL)
      throw std::runtime_error("gripper bar 1 is missing");

    return *bar1;
  // Bouml preserved body end 000E0FF1
}

YouBotGripperBar& YouBotGripper::getGripperBar2() {
  // Bouml preserved body begin 000E1071
    if (bar2 == NULL)
      throw std::runtime_error("gripper bar 2 is missing");

    return *bar2;
  // Bouml preserved body end 000E1071
}

void YouBotGripper::parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) const {
  // Bouml preserved body begin 00075C71
    std::stringstream errorMessageStream;
    errorMessageStream << "Joint " << this->jointNumber << ": ";
    std::string errorMessage;
    errorMessage = errorMessageStream.str();


    switch (mailboxMsg.stctInput.status) {
      case MAILBOX_SUCCESS:
        break;
      case INVALID_COMMAND:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << "; Command no: " << mailboxMsg.stctOutput.commandNumber << " is an invalid command!";
        //    throw JointParameterException(errorMessage + "invalid command");
        break;
      case WRONG_TYPE:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << " has a wrong type!";
        //    throw JointParameterException(errorMessage + "wrong type");
        break;
      case INVALID_VALUE:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << " Value: " << mailboxMsg.stctOutput.value << " is a invalid value!";
        //    throw JointParameterException(errorMessage + "invalid value");
        break;
      case CONFIGURATION_EEPROM_LOCKED:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << " Configuration EEPROM locked";
        //    throw JointParameterException(errorMessage + "configuration EEPROM locked");
        break;
      case COMMAND_NOT_AVAILABLE:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << "; Command no: " << mailboxMsg.stctOutput.commandNumber << "Command is not available!";
        //    throw JointParameterException(errorMessage + "command not available");
        break;
    }


  // Bouml preserved body end 00075C71
}

bool YouBotGripper::setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg) const {
  // Bouml preserved body begin 0005EF71

    YouBotSlaveMailboxMsg mailboxMsgBuffer;
    mailboxMsgBuffer = mailboxMsg;
    bool unvalid = true;
    unsigned int retry = 0;

    ethercatMaster->setMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber);
    //    LOG(trace) << "set Output CommandNumber " << (int) mailboxMsgBuffer.stctOutput.commandNumber
    //                  << " moduleAddress " << (int) mailboxMsgBuffer.stctOutput.moduleAddress
    //                  << " motorNumber " << (int) mailboxMsgBuffer.stctOutput.motorNumber
    //                  << " typeNumber " << (int) mailboxMsgBuffer.stctOutput.typeNumber
    //                  << " value " << mailboxMsgBuffer.stctOutput.value;

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {


      if (ethercatMaster->getMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber) &&
              mailboxMsgBuffer.stctInput.status == MAILBOX_SUCCESS) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
      //      LOG(trace) << "set Input CommandNumber " << (int) mailboxMsgBuffer.stctInput.commandNumber
      //                  << " moduleAddress " << (int) mailboxMsgBuffer.stctInput.moduleAddress
      //                  << " replyAddress " << (int) mailboxMsgBuffer.stctInput.replyAddress
      //                  << " status " << (int) mailboxMsgBuffer.stctInput.status
      //                  << " value " << mailboxMsgBuffer.stctInput.value;
    } while (retry < mailboxMsgRetries && unvalid);

    if (unvalid) {
      this->parseMailboxStatusFlags(mailboxMsgBuffer);
      return false;
    } else {
      return true;
    }

  // Bouml preserved body end 0005EF71
}

bool YouBotGripper::retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 0005EEF1

    bool unvalid = true;
    unsigned int retry = 0;

    ethercatMaster->setMailboxMsgBuffer(message, this->jointNumber);
    //     LOG(trace) << "get Output CommandNumber " << (int) message.stctOutput.commandNumber
    //                  << " moduleAddress " << (int) message.stctOutput.moduleAddress
    //                  << " motorNumber " << (int) message.stctOutput.motorNumber
    //                  << " typeNumber " << (int) message.stctOutput.typeNumber
    //                  << " value " << message.stctOutput.value
    //                  << " No " << this->jointNumber;

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {


      if (ethercatMaster->getMailboxMsgBuffer(message, this->jointNumber) &&
              message.stctInput.status == MAILBOX_SUCCESS) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
      //      LOG(trace) << "get input CommandNumber " << (int) message.stctInput.commandNumber
      //                 << " moduleAddress " << (int) message.stctInput.moduleAddress
      //                 << " replyAddress " << (int) message.stctInput.replyAddress
      //                 << " status " << (int) message.stctInput.status
      //                 << " value " << message.stctInput.value
      //                 << " No " << this->jointNumber;

    } while (retry < mailboxMsgRetries && unvalid);

    if (unvalid) {
      this->parseMailboxStatusFlags(message);
      return false;
    } else {
      return true;
    }

  // Bouml preserved body end 0005EEF1
}


} // namespace youbot
