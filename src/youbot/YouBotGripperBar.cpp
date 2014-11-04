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
#include "youbot_driver/youbot/YouBotGripperBar.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"
namespace youbot {

YouBotGripperBar::YouBotGripperBar(const unsigned int barNo, const unsigned int jointNo, const std::string& configFilePath) {
  // Bouml preserved body begin 000E0371
    this->jointNumber = jointNo;
    this->mailboxMsgRetries = 200;
    this->timeTillNextMailboxUpdate = 1; //ms
    this->barNo = barNo;
    this->maxTravelDistance = 0.0115 * meter;
    this->maxEncoderValue = 67000;
    this->barSpacingOffset = 0 * meter;

    ethercatMaster = &(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath));
  // Bouml preserved body end 000E0371
}

YouBotGripperBar::~YouBotGripperBar() {
  // Bouml preserved body begin 000E03F1
  // Bouml preserved body end 000E03F1
}

void YouBotGripperBar::setConfigurationParameter(const MaxEncoderValue& parameter) {
  // Bouml preserved body begin 00061E71
    this->maxEncoderValue = parameter.value;
  // Bouml preserved body end 00061E71
}

void YouBotGripperBar::getConfigurationParameter(MaxEncoderValue& parameter) const {
  // Bouml preserved body begin 000D7871
    parameter.value = this->maxEncoderValue;
  // Bouml preserved body end 000D7871
}

void YouBotGripperBar::getConfigurationParameter(MaxTravelDistance& parameter) const {
  // Bouml preserved body begin 000D77F1
    parameter.value = this->maxTravelDistance;
  // Bouml preserved body end 000D77F1
}

void YouBotGripperBar::setConfigurationParameter(const MaxTravelDistance& parameter) {
  // Bouml preserved body begin 00061DF1
    this->maxTravelDistance = parameter.value;
  // Bouml preserved body end 00061DF1
}

void YouBotGripperBar::setConfigurationParameter(const BarSpacingOffset& parameter) {
  // Bouml preserved body begin 00061871
    this->barSpacingOffset = parameter.value;
  // Bouml preserved body end 00061871
}

void YouBotGripperBar::getConfigurationParameter(BarSpacingOffset& parameter) const {
  // Bouml preserved body begin 000D7771
    parameter.value = this->barSpacingOffset;
  // Bouml preserved body end 000D7771
}

void YouBotGripperBar::setConfigurationParameter(const GripperBarName& parameter) {
  // Bouml preserved body begin 0010A271
    this->name = parameter.value;
  // Bouml preserved body end 0010A271
}

void YouBotGripperBar::getConfigurationParameter(GripperBarName& parameter) const {
  // Bouml preserved body begin 0010A1F1
    parameter.value = this->name;
  // Bouml preserved body end 0010A1F1
}

void YouBotGripperBar::getConfigurationParameter(YouBotGripperParameter& parameter) const {
  // Bouml preserved body begin 000E05F1
  
  if (parameter.getType() == MOTOR_CONTOLLER_PARAMETER) {

      YouBotSlaveMailboxMsg message;
      parameter.getYouBotMailboxMsg(message);
      message.stctOutput.commandNumber = GAP;
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.motorNumber = this->barNo;
      message.parameterName = parameter.getName();
      
      if (retrieveValueFromMotorContoller(message)) {
        parameter.setYouBotMailboxMsg(message);
      } else {
        throw JointParameterException("Unable to get parameter: " + parameter.getName() + " from the gripper");
      }
    }else{
      throw JointParameterException("Parameter " + parameter.getName() + " is not a motor controller parameter of the gripper");
    }
  // Bouml preserved body end 000E05F1
}

void YouBotGripperBar::setConfigurationParameter(const YouBotGripperParameter& parameter) {
  // Bouml preserved body begin 000E0671
   if (parameter.getType() == MOTOR_CONTOLLER_PARAMETER) {

      YouBotSlaveMailboxMsg message;
      parameter.getYouBotMailboxMsg(message);
      message.stctOutput.commandNumber = SAP;
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.motorNumber = this->barNo;
      message.parameterName = parameter.getName();
      
      if (!setValueToMotorContoller(message)) {
        throw JointParameterException("Unable to set parameter: " + parameter.getName() + " to the gripper");
      }
    }else{
      throw JointParameterException("Parameter " + parameter.getName() + " is not a motor controller parameter of the gripper");
    }
  // Bouml preserved body end 000E0671
}

void YouBotGripperBar::getConfigurationParameter(YouBotSlaveMailboxMsg& parameter) const {
  // Bouml preserved body begin 000E0A71
  if (!retrieveValueFromMotorContoller(parameter)) {
     throw JointParameterException("Unable to get parameter from the gripper");
   }
   this->parseMailboxStatusFlags(parameter);
  // Bouml preserved body end 000E0A71
}

void YouBotGripperBar::setData(const GripperBarEncoderSetpoint& encoderSetpoint) {
  // Bouml preserved body begin 000E0CF1
    YouBotSlaveMailboxMsg message;
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = MVP;
    message.stctOutput.typeNumber = 0; //move gripper absolute
    message.stctOutput.motorNumber = this->barNo;
    message.stctOutput.value = encoderSetpoint.barEncoder * -1;

    setValueToMotorContoller(message);
    
  // Bouml preserved body end 000E0CF1
}

void YouBotGripperBar::getData(GripperSensedVelocity& barVelocity) const {
  // Bouml preserved body begin 000E0DF1
   YouBotSlaveMailboxMsg message;
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = GAP; 
    message.stctOutput.typeNumber = 3; //actual velocity
    message.stctOutput.value = 0;
    
    message.stctOutput.motorNumber = this->barNo;

    retrieveValueFromMotorContoller(message);
    //std::cout << message.stctInput.value << std::endl;
    
    barVelocity.barVelocity = message.stctInput.value;

    
  // Bouml preserved body end 000E0DF1
}

void YouBotGripperBar::getData(GripperSensedBarPosition& barPosition) const {
  // Bouml preserved body begin 000F9171
    int valueBar = 0;
    ActualPosition actualPoseBar;
    this->getConfigurationParameter(actualPoseBar);
    actualPoseBar.getParameter(valueBar);

    barPosition.barPosition = (((double) valueBar / this->maxEncoderValue) * this->maxTravelDistance) + this->barSpacingOffset;

  // Bouml preserved body end 000F9171
}

void YouBotGripperBar::setData(GripperBarPositionSetPoint& barPosition) {
  // Bouml preserved body begin 000F91F1

    if (barPosition.barPosition > (this->maxTravelDistance + this->barSpacingOffset) || barPosition.barPosition < this->barSpacingOffset) {
      std::stringstream errorMessageStream;
      errorMessageStream << "The bar position is not allowed to be less than "<< this->barSpacingOffset.value() <<" or higher than " << (this->maxTravelDistance.value() + this->barSpacingOffset.value()) << ". You set " << barPosition.barPosition.value();
      throw std::out_of_range(errorMessageStream.str());
    }

    quantity<si::length> setpoint;;
    setpoint = (barPosition.barPosition - this->barSpacingOffset);
    
    GripperBarEncoderSetpoint setpointBar;
    setpointBar.barEncoder = setpoint / this->maxTravelDistance * this->maxEncoderValue;
    this->setData(setpointBar);

  // Bouml preserved body end 000F91F1
}

void YouBotGripperBar::parseGripperErrorFlags(const unsigned int& errosFlags) {
  // Bouml preserved body begin 00103CF1
    if (errosFlags & STALL_GUARD_STATUS) {
  //    LOG(warning) << "Gripper " << "stallguard2 threshold reached";
    }
    if (errosFlags & GRIPPER_OVER_TEMPERATURE) {
      LOG(error) << "Gripper " << "over temperature";
    }
    if (errosFlags & PRE_WARNING_OVER_TEMPERATURE) {
      LOG(warning) << "Gripper " << "pre warning over temperature";
    }
    if (errosFlags & SHORT_TO_GROUND_A) {
      LOG(error) << "Gripper " << "short to ground A";
    }
    if (errosFlags & SHORT_TO_GROUND_B) {
      LOG(error) << "Gripper " << "short to ground B";
    }
    if (errosFlags & OPEN_LOAD_A) {
      LOG(warning) << "Gripper " << "open load A";
    }
    if (errosFlags & OPEN_LOAD_B) {
      LOG(warning) << "Gripper " << "open load B";
    }
    if (errosFlags & STAND_STILL) {
 //     LOG(info) << "Gripper " << "stand still";
    }
    if ( !(errosFlags & STAND_STILL) && (errosFlags & STALL_GUARD_STATUS) ) {
      LOG(info) << "Gripper " << "motor stall";
    }
  // Bouml preserved body end 00103CF1
}

void YouBotGripperBar::parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) const {
  // Bouml preserved body begin 000E0E71
    std::stringstream errorMessageStream;
    errorMessageStream << "Joint " << this->jointNumber << ": ";
    std::string errorMessage;
    errorMessage = errorMessageStream.str();


    switch(mailboxMsg.stctInput.status){
      case MAILBOX_SUCCESS:
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
   

  // Bouml preserved body end 000E0E71
}

bool YouBotGripperBar::setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg) const {
  // Bouml preserved body begin 000E0EF1

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

  // Bouml preserved body end 000E0EF1
}

bool YouBotGripperBar::retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) const {
  // Bouml preserved body begin 000E0F71

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

  // Bouml preserved body end 000E0F71
}


} // namespace youbot
