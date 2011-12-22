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
namespace youbot {

YouBotGripper::YouBotGripper(const unsigned int jointNo, const std::string& configFilePath) {
  // Bouml preserved body begin 0005EFF1
    bar1 = NULL;
    bar2 = NULL;
    this->jointNumber = jointNo;
    this->mailboxMsgRetries = 200;
    this->timeTillNextMailboxUpdate = 1; //ms
    this->maxTravelDistance = 0.023 * meter;
    this->maxEncoderValue = 67000;
    this->barSpacingOffset = 0 * meter;
    ethercatMaster = &(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath));
    bar1 = new YouBotGripperBar(0, jointNo, configFilePath);
    bar2 = new YouBotGripperBar(1, jointNo, configFilePath);
  // Bouml preserved body end 0005EFF1
}

YouBotGripper::~YouBotGripper() {
  // Bouml preserved body begin 0005F071
    delete bar1;
    delete bar2;
    bar1 = NULL;
    bar2 = NULL;
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

    if (parameter.value) {
      LOG(info) << "Calibrate Gripper";


      StallGuard2Threshold threshold;
      threshold.setParameter(43);

      bar1->setConfigurationParameter(threshold);
      bar2->setConfigurationParameter(threshold);

      StallGuard2FilterEnable filter;
      filter.setParameter(false);
      bar1->setConfigurationParameter(filter);
      bar2->setConfigurationParameter(filter);

      StopOnStall stoponStall;
      stoponStall.setParameter(false);
      bar1->setConfigurationParameter(stoponStall);
      bar2->setConfigurationParameter(stoponStall);
      YouBotSlaveMailboxMsg message;


      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.commandNumber = MVP;
      message.stctOutput.typeNumber = 1; //move gripper relative
      message.stctOutput.value = this->maxEncoderValue;

      ActualLoadValue actualLoad;

      message.stctOutput.motorNumber = 0; //move bar 0
      setValueToMotorContoller(message);

      message.stctOutput.motorNumber = 1; //move bar 1
      setValueToMotorContoller(message);

      bar1->getConfigurationParameter(actualLoad);
      bar2->getConfigurationParameter(actualLoad);


      unsigned int loadbar1 = 1000;
      unsigned int loadbar2 = 1000;
      std::string parameterString;

      for (int i = 0; i < 40; i++) {
        bar1->getConfigurationParameter(actualLoad);
        actualLoad.getParameter(loadbar1);
        bar2->getConfigurationParameter(actualLoad);
        actualLoad.getParameter(loadbar2);
        if (loadbar1 < 400 || loadbar2 < 400)
          break;
        //   actualLoad.toString(parameterString);
        //   std::cout << parameterString << std::endl;
        SLEEP_MILLISEC(10);
      }

      //     SLEEP_MILLISEC(4000); //wait until the gripper is closed

      //stop Gripper motor
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.commandNumber = MST;

      message.stctOutput.value = 0;
      message.stctOutput.motorNumber = 0; //move bar 0
      setValueToMotorContoller(message);

      message.stctOutput.motorNumber = 1; //move bar 1
      setValueToMotorContoller(message);

      // set pose to zero as reference
      ActualPosition actualPose;
      actualPose.setParameter(0);
      bar1->setConfigurationParameter(actualPose);
      bar2->setConfigurationParameter(actualPose);

    }

  // Bouml preserved body end 00048271
}

void YouBotGripper::setConfigurationParameter(const BarSpacingOffset& parameter) {
  // Bouml preserved body begin 00061871
    this->barSpacingOffset = parameter.value;
  // Bouml preserved body end 00061871
}

void YouBotGripper::getConfigurationParameter(BarSpacingOffset& parameter) {
  // Bouml preserved body begin 000D7771
    parameter.value = this->barSpacingOffset;
  // Bouml preserved body end 000D7771
}

void YouBotGripper::setConfigurationParameter(const MaxTravelDistance& parameter) {
  // Bouml preserved body begin 00061DF1
    this->maxTravelDistance = parameter.value;
  // Bouml preserved body end 00061DF1
}

void YouBotGripper::getConfigurationParameter(MaxTravelDistance& parameter) {
  // Bouml preserved body begin 000D77F1
    parameter.value = this->maxTravelDistance;
  // Bouml preserved body end 000D77F1
}

void YouBotGripper::setConfigurationParameter(const MaxEncoderValue& parameter) {
  // Bouml preserved body begin 00061E71
    this->maxEncoderValue = parameter.value;
  // Bouml preserved body end 00061E71
}

void YouBotGripper::getConfigurationParameter(MaxEncoderValue& parameter) {
  // Bouml preserved body begin 000D7871
    parameter.value = this->maxEncoderValue;
  // Bouml preserved body end 000D7871
}

void YouBotGripper::getConfigurationParameter(YouBotSlaveMailboxMsg& parameter) {
  // Bouml preserved body begin 000DE971
    if (!retrieveValueFromMotorContoller(parameter)) {
      throw JointParameterException("Unable to get parameter from the gripper");
    }
    this->parseMailboxStatusFlags(parameter);
  // Bouml preserved body end 000DE971
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

    if (barSpacing.barSpacing > (maxTravelDistance + barSpacingOffset) || barSpacing.barSpacing < barSpacingOffset) {
      std::stringstream errorMessageStream;
      errorMessageStream << "The bar spacing is not allowed to be less than 0 m or higher than " << (maxTravelDistance + barSpacingOffset);
      throw std::out_of_range(errorMessageStream.str());
    }

    GripperBarEncoterSetpoint setpointBar1;
    GripperBarEncoterSetpoint setpointBar2;

    setpointBar1.barEncoder = (barSpacing.barSpacing + barSpacingOffset) / maxTravelDistance * maxEncoderValue;
    setpointBar2.barEncoder = (barSpacing.barSpacing + barSpacingOffset) / maxTravelDistance * maxEncoderValue;


    bar1->setData(setpointBar1);
    bar2->setData(setpointBar2);

  // Bouml preserved body end 0005F8F1
}

void YouBotGripper::getData(GripperSensedBarSpacing& barSpacing) {
  // Bouml preserved body begin 0005F971

    int valueBar1 = 0;
    int valueBar2 = 0;

    ActualPosition actualPoseBar1;
    ActualPosition actualPoseBar2;

    bar1->getConfigurationParameter(actualPoseBar1);
    bar2->getConfigurationParameter(actualPoseBar2);
    actualPoseBar1.getParameter(valueBar1);
    actualPoseBar2.getParameter(valueBar2);


    quantity<si::length> bar1Pose;
    quantity<si::length> bar2Pose;
    bar1Pose = (((double) valueBar1 / maxEncoderValue) * (maxTravelDistance / 2.0)) + barSpacingOffset;

    bar2Pose = (((double) valueBar2 / maxEncoderValue) * (maxTravelDistance / 2.0)) + barSpacingOffset;

    barSpacing.barSpacing = bar1Pose + bar2Pose;


  // Bouml preserved body end 0005F971
}

void YouBotGripper::open() {
  // Bouml preserved body begin 000E3BF1
    GripperBarSpacingSetPoint barSpacing;
    barSpacing.barSpacing = 0.023 * meter;

    this->setData(barSpacing);

  // Bouml preserved body end 000E3BF1
}

bool YouBotGripper::closeUntilMaxForce() {
  // Bouml preserved body begin 000E3C71
    StallGuard2Threshold threshold;
    threshold.setParameter(43);

    bar1->setConfigurationParameter(threshold);
    bar2->setConfigurationParameter(threshold);

    StallGuard2FilterEnable filter;
    filter.setParameter(false);
    bar1->setConfigurationParameter(filter);
    bar2->setConfigurationParameter(filter);

    StopOnStall stoponStall;
    stoponStall.setParameter(false);
    bar1->setConfigurationParameter(stoponStall);
    bar2->setConfigurationParameter(stoponStall);
    YouBotSlaveMailboxMsg message;


    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = MVP;
    message.stctOutput.typeNumber = 1; //move gripper relative
    message.stctOutput.value = this->maxEncoderValue;

    ActualLoadValue actualLoad;

    message.stctOutput.motorNumber = 0; //move bar 0
    setValueToMotorContoller(message);

    message.stctOutput.motorNumber = 1; //move bar 1
    setValueToMotorContoller(message);

    bar1->getConfigurationParameter(actualLoad);
    bar2->getConfigurationParameter(actualLoad);


    unsigned int loadbar1 = 1000;
    unsigned int loadbar2 = 1000;
    std::string parameterString;

    for (int i = 0; i < 40; i++) {
      bar1->getConfigurationParameter(actualLoad);
      actualLoad.getParameter(loadbar1);
      bar2->getConfigurationParameter(actualLoad);
      actualLoad.getParameter(loadbar2);
      if (loadbar1 < 400 || loadbar2 < 400)
        break;
      //   actualLoad.toString(parameterString);
      //   std::cout << parameterString << std::endl;
      SLEEP_MILLISEC(10);
    }

    //     SLEEP_MILLISEC(4000); //wait until the gripper is closed

    //stop Gripper motor
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = MST;

    message.stctOutput.value = 0;
    message.stctOutput.motorNumber = 0; //move bar 0
    setValueToMotorContoller(message);

    message.stctOutput.motorNumber = 1; //move bar 1
    setValueToMotorContoller(message);

  // Bouml preserved body end 000E3C71
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

void YouBotGripper::parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 00075C71
    std::stringstream errorMessageStream;
    errorMessageStream << "Joint " << this->jointNumber << ": ";
    std::string errorMessage;
    errorMessage = errorMessageStream.str();


    switch (mailboxMsg.stctInput.status) {
      case NO_ERROR:
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
