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
#include "youbot/YouBotGripper.hpp"
#include "youbot/EthercatMaster.hpp"
namespace youbot {

YouBotGripper::YouBotGripper(const unsigned int jointNo) {
  // Bouml preserved body begin 0005EFF1
    this->jointNumber = jointNo;
    this->mailboxMsgRetries = 100;
    this->timeTillNextMailboxUpdate = 1; //ms
    this->maxTravelDistance = 0.023 * meter;
    this->maxEncoderValue = 67000;
    this->barSpacingOffset = 0 * meter;
    this->lastGripperPosition = 0 * meter;
  // Bouml preserved body end 0005EFF1
}

YouBotGripper::~YouBotGripper() {
  // Bouml preserved body begin 0005F071
  // Bouml preserved body end 0005F071
}

void YouBotGripper::getConfigurationParameter(GripperParameter& parameter) {
  // Bouml preserved body begin 0005FBF1
  throw std::runtime_error("Please use YouBotGripperParameter");
  // Bouml preserved body end 0005FBF1
}

void YouBotGripper::setConfigurationParameter(const GripperParameter& parameter) {
  // Bouml preserved body begin 0005FA71
  throw std::runtime_error("Please use YouBotGripperParameter");
  // Bouml preserved body end 0005FA71
}

void YouBotGripper::getConfigurationParameter(YouBotGripperParameter& parameter) {
  // Bouml preserved body begin 000482F1
  LOG(info) << "Nothing to do";
  // Bouml preserved body end 000482F1
}

void YouBotGripper::setConfigurationParameter(const YouBotGripperParameter& parameter) {
  // Bouml preserved body begin 000617F1
   LOG(info) << "Nothing to do";
  // Bouml preserved body end 000617F1
}

void YouBotGripper::setConfigurationParameter(const CalibrateGripper& parameter) {
  // Bouml preserved body begin 00048271
    
      if (parameter.value) {
        YouBotSlaveMailboxMsg message;

        LOG(info) << "Calibrate gripper";
        message.stctOutput.moduleAddress = GRIPPER;
        message.stctOutput.commandNumber = MVP;
        message.stctOutput.typeNumber = 1; //move gripper
        message.stctOutput.motorNumber = 0; //always zero
        message.stctOutput.value = this->maxEncoderValue;

        setValueToMotorContoller(message);
        
        SLEEP_MILLISEC(4000); //wait until the gripper ist closed

        //set encoder reference to zero
        YouBotSlaveMsg messageBuffer;
        messageBuffer.stctOutput.controllerMode = MOTOR_STOP;
        messageBuffer.stctOutput.positionOrSpeed = 0;
        EthercatMaster::getInstance().setMsgBuffer(messageBuffer, this->jointNumber);
        this->lastGripperPosition = 0 * meter;

        //stop Gripper motor
        SLEEP_MILLISEC(timeTillNextMailboxUpdate*2);
        message.stctOutput.value = 0;
        setValueToMotorContoller(message);
      }
    
  // Bouml preserved body end 00048271
}

void YouBotGripper::setConfigurationParameter(const BarSpacingOffset& parameter) {
  // Bouml preserved body begin 00061871
      this->barSpacingOffset = parameter.value;
  // Bouml preserved body end 00061871
}

void YouBotGripper::setConfigurationParameter(const MaxTravelDistance& parameter) {
  // Bouml preserved body begin 00061DF1
  this->maxTravelDistance = parameter.value;
  // Bouml preserved body end 00061DF1
}

void YouBotGripper::setConfigurationParameter(const MaxEncoderValue& parameter) {
  // Bouml preserved body begin 00061E71
  this->maxEncoderValue = parameter.value;
  // Bouml preserved body end 00061E71
}

void YouBotGripper::getData(const GripperData& data) {
  // Bouml preserved body begin 0005FB71
    LOG(info) << "Nothing to do";
  // Bouml preserved body end 0005FB71
}

void YouBotGripper::setData(const GripperData& data) {
  // Bouml preserved body begin 0005FAF1
    LOG(info) << "Nothing to do";
  // Bouml preserved body end 0005FAF1
}

void YouBotGripper::getData(OneDOFGripperData& data) {
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

  if(barSpacing.barSpacing > (maxTravelDistance + barSpacingOffset) || barSpacing.barSpacing < barSpacingOffset){
    std::stringstream errorMessageStream;
    errorMessageStream << "The bar spacing is not allowd to be less than 0 m or higher than " << (maxTravelDistance + barSpacingOffset);
    throw std::out_of_range(errorMessageStream.str());
  }
    YouBotSlaveMailboxMsg message;
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = MVP;
    message.stctOutput.typeNumber = 1; //move gripper
    message.stctOutput.motorNumber = 0; //always zero
    message.stctOutput.value = (int)(((this->lastGripperPosition - barSpacing.barSpacing)+barSpacingOffset)/maxTravelDistance * maxEncoderValue);

    this->lastGripperPosition = barSpacing.barSpacing;
    setValueToMotorContoller(message);

  // Bouml preserved body end 0005F8F1
}

void YouBotGripper::getData(GripperBarSpacingSetPoint& barSpacing) {
  // Bouml preserved body begin 0005F971
   LOG(info) << "At the moment it is not possible to get the sensed position of the gripper";
  // Bouml preserved body end 0005F971
}

void YouBotGripper::parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 00075C71
    std::stringstream errorMessageStream;
    errorMessageStream << "Joint " << this->jointNumber << ": ";
    std::string errorMessage;
    errorMessage = errorMessageStream.str();


    switch(mailboxMsg.stctInput.status){
      case NO_ERROR:
        break;
      case INVALID_COMMAND:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << "; Command no: " << mailboxMsg.stctOutput.commandNumber << " is an invalid command!" ;
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

bool YouBotGripper::setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 0005EF71

    YouBotSlaveMailboxMsg mailboxMsgBuffer;
    mailboxMsgBuffer = mailboxMsg;
    bool unvalid = true;
    unsigned int retry = 0;

    EthercatMaster::getInstance().setMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber);

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
      EthercatMaster::getInstance().getMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber);
      /*    LOG(trace) << "CommandNumber " << (int) mailboxMsgBuffer.stctInput.commandNumber
                  << " moduleAddress " << (int) mailboxMsgBuffer.stctInput.moduleAddress
                  << " replyAddress " << (int) mailboxMsgBuffer.stctInput.replyAddress
                  << " status " << (int) mailboxMsgBuffer.stctInput.status
                  << " value " << mailboxMsgBuffer.stctInput.value;
       */
      if (mailboxMsgBuffer.stctOutput.commandNumber == mailboxMsgBuffer.stctInput.commandNumber &&
              mailboxMsgBuffer.stctInput.status == NO_ERROR) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
    } while (retry < mailboxMsgRetries && unvalid);

    if (unvalid) {
      this->parseMailboxStatusFlags(mailboxMsgBuffer);
      return false;
    } else {
      return true;
    }

  // Bouml preserved body end 0005EF71
}

bool YouBotGripper::retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 0005EEF1

    bool unvalid = true;
    unsigned int retry = 0;

    EthercatMaster::getInstance().setMailboxMsgBuffer(message, this->jointNumber);

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
      EthercatMaster::getInstance().getMailboxMsgBuffer(message, this->jointNumber);
      /*   LOG(trace) << "CommandNumber " << (int) message.stctInput.commandNumber
                 << " moduleAddress " << (int) message.stctInput.moduleAddress
                 << " replyAddress " << (int) message.stctInput.replyAddress
                 << " status " << (int) message.stctInput.status
                 << " value " << message.stctInput.value;
       */
      if (message.stctOutput.commandNumber == message.stctInput.commandNumber &&
              message.stctInput.status == NO_ERROR) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
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
