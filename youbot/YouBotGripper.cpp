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
#ifdef ETHERCAT_MASTER_WITHOUT_THREAD
  #include "youbot/EthercatMasterWithoutThread.hpp"
#else
  #include "youbot/EthercatMaster.hpp"
#endif
namespace youbot {

YouBotGripper::YouBotGripper(const unsigned int jointNo, const std::string& configFilePath) {
  // Bouml preserved body begin 0005EFF1
    this->jointNumber = jointNo;
    this->mailboxMsgRetries = 200;
    this->timeTillNextMailboxUpdate = 1; //ms
    this->maxTravelDistance = 0.023/2.0 * meter;
    this->maxEncoderValue = 67000;
    this->barSpacingOffset = 0 * meter;
    this->lastGripperPosition = 0 * meter;
    ethercatMaster = &(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath));
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

void YouBotGripper::getConfigurationParameter(GripperFirmwareVersion& parameter) {
  // Bouml preserved body begin 000BEF71

    YouBotSlaveMailboxMsg message;
    parameter.getYouBotMailboxMsg(message);

    bool unvalid = true;
    unsigned int retry = 0;

    ethercatMaster->setMailboxMsgBuffer(message, this->jointNumber);
    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
      if( ethercatMaster->getMailboxMsgBuffer(message, this->jointNumber) ) {
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
    versionString[0] = message.stctInput.replyAddress;
    versionString[1] = message.stctInput.moduleAddress;
    versionString[2] = message.stctInput.status;
    versionString[3] = message.stctInput.commandNumber;
    versionString[4] = message.stctInput.value >> 24;
    versionString[5] = message.stctInput.value >> 16;
    versionString[6] = message.stctInput.value >> 8;
    versionString[7] = message.stctInput.value & 0xff;
    
   // LOG(trace) << versionString;
    int controllerType = 0;
    float firmwareVersion = 0;
    sscanf (versionString,"KR%dV%f",&controllerType,&firmwareVersion); //KR842V20

    
    parameter.setParameter(controllerType, firmwareVersion);

    return;
  // Bouml preserved body end 000BEF71
}

void YouBotGripper::getConfigurationParameter(YouBotGripperParameter& parameter, const BarNumber& barNumber) {
  // Bouml preserved body begin 000482F1
  if (parameter.getType() == MOTOR_CONTOLLER_PARAMETER) {

      YouBotSlaveMailboxMsg message;
      parameter.getYouBotMailboxMsg(message);
      message.stctOutput.commandNumber = GAP;
      message.stctOutput.moduleAddress = GRIPPER;
      if(barNumber == BAR_ONE){
        message.stctOutput.motorNumber = 0;
      }else if(barNumber == BAR_TWO){
        message.stctOutput.motorNumber = 1;
      }else{
        throw JointParameterException("Invalid bar number");
      }

      message.parameterName = parameter.getName();
      if (retrieveValueFromMotorContoller(message)) {
        parameter.setYouBotMailboxMsg(message);
      } else {
        throw JointParameterException("Unable to get parameter: " + parameter.getName() + " from the gripper");
      }
      SLEEP_MILLISEC(10);
    }
  // Bouml preserved body end 000482F1
}

void YouBotGripper::setConfigurationParameter(const YouBotGripperParameter& parameter, const BarNumber& barNumber) {
  // Bouml preserved body begin 000617F1
   if (parameter.getType() == MOTOR_CONTOLLER_PARAMETER) {

      YouBotSlaveMailboxMsg message;
      parameter.getYouBotMailboxMsg(message);
      message.stctOutput.commandNumber = SAP;
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.motorNumber = barNumber;

      message.parameterName = parameter.getName();
      if (!setValueToMotorContoller(message)) {
        throw JointParameterException("Unable to set parameter: " + parameter.getName() + " to the gripper");
      }
      SLEEP_MILLISEC(10);
    }
  // Bouml preserved body end 000617F1
}

void YouBotGripper::setConfigurationParameter(const CalibrateGripper& parameter) {
  // Bouml preserved body begin 00048271
    
      if (parameter.value) {
        
        StopOnStall stopStall;
        stopStall.setParameter(true);
    //    this->setConfigurationParameter(stopStall, BAR_ONE);
    //    this->setConfigurationParameter(stopStall, BAR_TWO);

        /*
        StallGuard2Threshold sgThreshold;
        sgThreshold.setParameter(0);
        this->setConfigurationParameter(sgThreshold, BAR_ONE);
        this->setConfigurationParameter(sgThreshold, BAR_TWO);
        this->getConfigurationParameter(sgThreshold, BAR_ONE);
   
        
        SmartEnergyHysteresis hysteresis;
        this->getConfigurationParameter(hysteresis, BAR_TWO);
        hysteresis.getParameter(dummy);
        LOG(trace) << "hysteresis 2 "<< dummy;
        */
        YouBotSlaveMailboxMsg message;

        LOG(info) << "Calibrate Gripper";
        message.stctOutput.moduleAddress = GRIPPER;
        message.stctOutput.commandNumber = MVP;
        message.stctOutput.typeNumber = 1; //move gripper relative
        message.stctOutput.value = this->maxEncoderValue;


        message.stctOutput.motorNumber = 0; //move bar 0
        setValueToMotorContoller(message);
        SLEEP_MILLISEC(10);
        message.stctOutput.motorNumber = 1; //move bar 1
        setValueToMotorContoller(message);
        SLEEP_MILLISEC(10);
        
        SLEEP_MILLISEC(4000); //wait until the gripper is closed

        
        this->lastGripperPosition = 0 * meter;

        //stop Gripper motor
        message.stctOutput.value = 0;
        message.stctOutput.motorNumber = 0; //move bar 0
        setValueToMotorContoller(message);
        SLEEP_MILLISEC(10);
        message.stctOutput.motorNumber = 1; //move bar 1
        setValueToMotorContoller(message);
        SLEEP_MILLISEC(10);
        stopStall.setParameter(false);
     //   this->setConfigurationParameter(stopStall, BAR_ONE);
    //    this->setConfigurationParameter(stopStall, BAR_TWO);


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
    message.stctOutput.typeNumber = 1; //move gripper relative
    message.stctOutput.value = (int)(((this->lastGripperPosition - barSpacing.barSpacing)+barSpacingOffset)/maxTravelDistance * maxEncoderValue);

    this->lastGripperPosition = barSpacing.barSpacing;

    message.stctOutput.motorNumber = 0; //move bar 0
    setValueToMotorContoller(message);
    SLEEP_MILLISEC(10);
    message.stctOutput.motorNumber = 1; //move bar 1
    setValueToMotorContoller(message);
    SLEEP_MILLISEC(10);

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

    ethercatMaster->setMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber);
//    LOG(trace) << "set Output CommandNumber " << (int) mailboxMsgBuffer.stctOutput.commandNumber
//                  << " moduleAddress " << (int) mailboxMsgBuffer.stctOutput.moduleAddress
//                  << " motorNumber " << (int) mailboxMsgBuffer.stctOutput.motorNumber
//                  << " typeNumber " << (int) mailboxMsgBuffer.stctOutput.typeNumber
//                  << " value " << mailboxMsgBuffer.stctOutput.value;

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
          
       
      if (ethercatMaster->getMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber) &&
          mailboxMsgBuffer.stctInput.status == NO_ERROR) {
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

bool YouBotGripper::retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) {
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
          message.stctInput.status == NO_ERROR) {
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
