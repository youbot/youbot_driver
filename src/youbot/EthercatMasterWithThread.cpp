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
extern "C" {
#include "youbot_driver/soem/ethercattype.h"
#include "nicdrv.h"
#include "youbot_driver/soem/ethercatbase.h"
#include "youbot_driver/soem/ethercatmain.h"
#include "youbot_driver/soem/ethercatconfig.h"
#include "youbot_driver/soem/ethercatcoe.h"
#include "youbot_driver/soem/ethercatdc.h"
#include "youbot_driver/soem/ethercatprint.h"
}
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

namespace youbot {

EthercatMasterWithThread::EthercatMasterWithThread(const std::string& configFile, const std::string& configFilePath) {
  // Bouml preserved body begin 00041171

    this->ethercatConnectionEstablished = false;
    ethernetDevice = "eth0";
    timeTillNextEthercatUpdate = 1000; //usec
    mailboxTimeout = 4000; //micro sec
    ethercatTimeout = 500; //micro sec
    communicationErrors = 0;
    maxCommunicationErrors = 100;
    stopThread = false;
    this->automaticSendOn = true;
    this->automaticReceiveOn = true;
    this->configFileName = configFile;
    this->configFilepath = configFilePath;
    configfile = NULL;

    //initialize to zero
    for (unsigned int i = 0; i < 4096; i++) {
      IOmap_[i] = 0;
    }
    //read ethercat parameters form config file
    configfile = new ConfigFile(this->configFileName, this->configFilepath);

    // configfile.setSection("EtherCAT");
    configfile->readInto(ethernetDevice, "EtherCAT", "EthernetDevice");
    configfile->readInto(timeTillNextEthercatUpdate, "EtherCAT", "EtherCATUpdateRate_[usec]");
    configfile->readInto(ethercatTimeout, "EtherCAT", "EtherCATTimeout_[usec]");
    configfile->readInto(mailboxTimeout, "EtherCAT", "MailboxTimeout_[usec]");
    configfile->readInto(maxCommunicationErrors, "EtherCAT", "MaximumNumberOfEtherCATErrors");

    this->initializeEthercat();


  // Bouml preserved body end 00041171
}

EthercatMasterWithThread::~EthercatMasterWithThread() {
  // Bouml preserved body begin 000411F1
    stopThread = true;
    threads.join_all();
    this->closeEthercat();
    if (configfile != NULL)
      delete configfile;
  // Bouml preserved body end 000411F1
}

bool EthercatMasterWithThread::isThreadActive() {
  // Bouml preserved body begin 000E6AF1
    return true;
  // Bouml preserved body end 000E6AF1
}

///return the quantity of ethercat slave which have an input/output buffer
unsigned int EthercatMasterWithThread::getNumberOfSlaves() const {
  // Bouml preserved body begin 00044A71
    return this->nrOfSlaves;
  // Bouml preserved body end 00044A71
}

void EthercatMasterWithThread::AutomaticSendOn(const bool enableAutomaticSend) {
  // Bouml preserved body begin 000775F1
    this->automaticSendOn = enableAutomaticSend;

    if (this->automaticSendOn == true) {
      unsigned int slaveNo = 0;

      for (unsigned int i = 0; i < automaticSendOffBufferVector.size(); i++) {
        slaveNo = automaticSendOffBufferVector[i].jointNumber - 1;
        slaveMessages[slaveNo].stctInput.Set(automaticSendOffBufferVector[i].stctInput);
        slaveMessages[slaveNo].stctOutput.Set(automaticSendOffBufferVector[i].stctOutput);
        slaveMessages[slaveNo].jointNumber.Set(automaticSendOffBufferVector[i].jointNumber);
      }

      automaticSendOffBufferVector.clear();
    } else {
      return;
    }
    return;
  // Bouml preserved body end 000775F1
}

void EthercatMasterWithThread::AutomaticReceiveOn(const bool enableAutomaticReceive) {
  // Bouml preserved body begin 0008FB71
    this->automaticReceiveOn = enableAutomaticReceive;


    if (this->automaticReceiveOn == false) {
      

      for (unsigned int i = 0; i < slaveMessages.size(); i++) {
        slaveMessages[i].stctInput.Get(automaticReceiveOffBufferVector[i].stctInput);
        slaveMessages[i].stctOutput.Get(automaticReceiveOffBufferVector[i].stctOutput);
        slaveMessages[i].jointNumber.Get(automaticReceiveOffBufferVector[i].jointNumber);
      }
    }

    return;
  // Bouml preserved body end 0008FB71
}

///provides all ethercat slave informations from the SOEM driver
///@param ethercatSlaveInfos ethercat slave informations
void EthercatMasterWithThread::getEthercatDiagnosticInformation(std::vector<ec_slavet>& ethercatSlaveInfos) {
  // Bouml preserved body begin 00061EF1
    ethercatSlaveInfos = this->ethercatSlaveInfo;
    for (unsigned int i = 0; i < ethercatSlaveInfos.size(); i++) {
      ethercatSlaveInfos[i].inputs = NULL;
      ethercatSlaveInfos[i].outputs = NULL;
    }
  // Bouml preserved body end 00061EF1
}

///sends ethercat messages to the motor controllers
/// returns a true if everything it OK and returns false if something fail
bool EthercatMasterWithThread::sendProcessData() {
  // Bouml preserved body begin 000E68F1
    throw std::runtime_error("When using the EthercatMaster with thread there is not need to send process data manual.");
    return false;
  // Bouml preserved body end 000E68F1
}

/// receives ethercat messages from the motor controllers
/// returns a true if everything it OK and returns false if something fail
bool EthercatMasterWithThread::receiveProcessData() {
  // Bouml preserved body begin 000E6971
    throw std::runtime_error("When using the EthercatMaster with thread there is not need to receive process data manual");
    return false;
  // Bouml preserved body end 000E6971
}

/// checks if an error has occurred in the soem driver
/// returns a true if an error has occurred
bool EthercatMasterWithThread::isErrorInSoemDriver() {
  // Bouml preserved body begin 000E69F1

    return ec_iserror();

  // Bouml preserved body end 000E69F1
}

void EthercatMasterWithThread::registerJointTrajectoryController(JointTrajectoryController* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 000EBCF1
    {
      boost::mutex::scoped_lock trajectoryControllerMutex(trajectoryControllerVectorMutex);
      if (this->trajectoryControllers[JointNumber - 1] != NULL)
        throw std::runtime_error("A joint trajectory controller is already register for this joint!");
      if ((JointNumber - 1) >= this->trajectoryControllers.size())
        throw std::out_of_range("Invalid joint number");

      this->trajectoryControllers[JointNumber - 1] = object;
    }
    LOG(debug) << "register joint trajectory controller for joint: " << JointNumber;
  // Bouml preserved body end 000EBCF1
}

void EthercatMasterWithThread::deleteJointTrajectoryControllerRegistration(const unsigned int JointNumber) {
  // Bouml preserved body begin 000F06F1
    {
      boost::mutex::scoped_lock trajectoryControllerMutex(trajectoryControllerVectorMutex);
      if ((JointNumber - 1) >= this->trajectoryControllers.size())
        throw std::out_of_range("Invalid joint number");

      this->trajectoryControllers[JointNumber - 1] = NULL;
    }
    LOG(debug) << "delete joint trajectory controller registration for joint: " << JointNumber;
  // Bouml preserved body end 000F06F1
}

unsigned int EthercatMasterWithThread::getNumberOfThreadCyclesPerSecond() {
  // Bouml preserved body begin 000F41F1

    return static_cast<unsigned int> (1.0 / ((double) timeTillNextEthercatUpdate / 1000 / 1000));
  // Bouml preserved body end 000F41F1
}

bool EthercatMasterWithThread::isEtherCATConnectionEstablished() {
  // Bouml preserved body begin 000F7771
    return this->ethercatConnectionEstablished;
  // Bouml preserved body end 000F7771
}

void EthercatMasterWithThread::registerJointLimitMonitor(JointLimitMonitor* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 000FB071
    {
      boost::mutex::scoped_lock limitMonitorMutex(jointLimitMonitorVectorMutex);
      if (this->jointLimitMonitors[JointNumber - 1] != NULL)
        LOG(warning) << "A joint limit monitor is already register for this joint!";
      if ((JointNumber - 1) >= this->jointLimitMonitors.size())
        throw std::out_of_range("Invalid joint number");

      this->jointLimitMonitors[JointNumber - 1] = object;
    }
    LOG(debug) << "register a joint limit monitor for joint: " << JointNumber;
  // Bouml preserved body end 000FB071
}

void EthercatMasterWithThread::registerDataTrace(void* object, const unsigned int JointNumber) {
  // Bouml preserved body begin 00105871
    {
      boost::mutex::scoped_lock datatraceM(dataTracesMutex);
      if (this->dataTraces[JointNumber - 1] != NULL)
        throw std::runtime_error("A data trace is already register for this joint!");
      if ((JointNumber - 1) >= this->dataTraces.size())
        throw std::out_of_range("Invalid joint number");

      this->dataTraces[JointNumber - 1] = (DataTrace*)object;
    }
    LOG(debug) << "register a data trace for joint: " << JointNumber;
  // Bouml preserved body end 00105871
}

void EthercatMasterWithThread::deleteDataTraceRegistration(const unsigned int JointNumber) {
  // Bouml preserved body begin 001058F1
    {
      boost::mutex::scoped_lock datatraceM(dataTracesMutex);
      if ((JointNumber - 1) >= this->dataTraces.size())
        throw std::out_of_range("Invalid joint number");

      this->dataTraces[JointNumber - 1] = NULL;

    }
    LOG(debug) << "removed data trace for joint: " << JointNumber;
  // Bouml preserved body end 001058F1
}

///establishes the ethercat connection
void EthercatMasterWithThread::initializeEthercat() {
  // Bouml preserved body begin 000410F1

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(const_cast<char*>(ethernetDevice.c_str()))) {
      LOG(info) << "Initializing EtherCAT on " << ethernetDevice << " with communication thread";
      /* find and auto-config slaves */
      if (ec_config(TRUE, &IOmap_) > 0) {

        LOG(trace) << ec_slavecount << " EtherCAT slaves found and configured.";

        /* wait for all slaves to reach Pre OP state */
        /*ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
        if (ec_slave[0].state != EC_STATE_PRE_OP ){
        LOG(debug) << "Not all slaves reached pre operational state.";
        ec_readstate();
        //If not all slaves operational find out which one
          for(int i = 1; i<=ec_slavecount ; i++){
            if(ec_slave[i].state != EC_STATE_PRE_OP){
              printf("Slave %d State=%2x StatusCode=%4x : %s\n",
              i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
          }
        }
         */

        /* distributed clock is not working
        //Configure distributed clock
        if(!ec_configdc()){
          LOG(warning) << "no distributed clock is available";
        }else{

          uint32 CyclTime = 4000000;
          uint32 CyclShift = 0;
          for (int i = 1; i <= ec_slavecount; i++) {
            ec_dcsync0(i, true, CyclTime, CyclShift);
          }

        }
         */

        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        if (ec_slave[0].state != EC_STATE_SAFE_OP) {
          LOG(warning) << "Not all EtherCAT slaves reached safe operational state.";
          ec_readstate();
          //If not all slaves operational find out which one
          for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
              LOG(info) << "Slave " << i << " State=" << ec_slave[i].state << " StatusCode=" << ec_slave[i].ALstatuscode << " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode);

            }
          }
        }


        //Read the state of all slaves
        //ec_readstate();

        LOG(trace) << "Request operational state for all EtherCAT slaves";

        ec_slave[0].state = EC_STATE_OPERATIONAL;
        // request OP state for all slaves
        /* send one valid process data to make outputs in slaves happy*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for all slaves */
        ec_writestate(0);
        // wait for all slaves to reach OP state

        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
          LOG(trace) << "Operational state reached for all EtherCAT slaves.";
        } else {
          throw std::runtime_error("Not all EtherCAT slaves reached operational state.");

        }

      } else {
        throw std::runtime_error("No EtherCAT slaves found!");
      }

    } else {
      throw std::runtime_error("No socket connection on " + ethernetDevice + "\nExcecute as root");
    }



    std::string baseJointControllerName = "TMCM-174";
    std::string baseJointControllerNameAlternative = "TMCM-1632";
    std::string manipulatorJointControllerName = "TMCM-174";
    std::string ManipulatorJointControllerNameAlternative = "TMCM-1610";
    std::string actualSlaveName;
    nrOfSlaves = 0;
    YouBotSlaveMsg emptySlaveMsg;
    YouBotSlaveMsgThreadSafe emptySlaveMsgThreadSafe;

    configfile->readInto(baseJointControllerName, "BaseJointControllerName");
    configfile->readInto(baseJointControllerNameAlternative, "BaseJointControllerNameAlternative");
    configfile->readInto(manipulatorJointControllerName, "ManipulatorJointControllerName");
    configfile->readInto(ManipulatorJointControllerNameAlternative, "ManipulatorJointControllerNameAlternative");

    //reserve memory for all slave with a input/output buffer
    for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
           LOG(trace) << "Slave: " << cnt  << " Name: " << ec_slave[cnt].name  << " Output size: " << ec_slave[cnt].Obits
                   << "bits Input size: " << ec_slave[cnt].Ibits << "bits State: " << ec_slave[cnt].state  
                   << " delay: " << ec_slave[cnt].pdelay; //<< " has dclock: " << (bool)ec_slave[cnt].hasdc;

      ethercatSlaveInfo.push_back(ec_slave[cnt]);

      actualSlaveName = ec_slave[cnt].name;
      if ((actualSlaveName == baseJointControllerName || actualSlaveName == baseJointControllerNameAlternative || 
              actualSlaveName == manipulatorJointControllerName || actualSlaveName == ManipulatorJointControllerNameAlternative
              ) && ec_slave[cnt].Obits > 0 && ec_slave[cnt].Ibits > 0) {
        nrOfSlaves++;
        ethercatOutputBufferVector.push_back((SlaveMessageOutput*) (ec_slave[cnt].outputs));
        ethercatInputBufferVector.push_back((SlaveMessageInput*) (ec_slave[cnt].inputs));
        YouBotSlaveMailboxMsgThreadSafe emptyMailboxSlaveMsg(cnt);
        mailboxMessages.push_back(emptyMailboxSlaveMsg);
        pendingMailboxMsgsReply.push_back(false);
        trajectoryControllers.push_back(NULL);
        jointLimitMonitors.push_back(NULL);
        slaveMessages.push_back(emptySlaveMsgThreadSafe);
        outstandingMailboxMsgFlag.push_back(false);
        newInputMailboxMsgFlag.push_back(false);
        dataTraces.push_back(NULL);
      }
    }
    automaticReceiveOffBufferVector.resize(slaveMessages.size());

    if (nrOfSlaves > 0) {
      LOG(info) << nrOfSlaves << " EtherCAT slaves found";
    } else {
      throw std::runtime_error("No EtherCAT slave could be found");
      return;
    }

    stopThread = false;
    threads.create_thread(boost::bind(&EthercatMasterWithThread::updateSensorActorValues, this));

    SLEEP_MILLISEC(10); //needed to start up thread and EtherCAT communication

    this->ethercatConnectionEstablished = true;
    return;
  // Bouml preserved body end 000410F1
}

///closes the ethercat connection
bool EthercatMasterWithThread::closeEthercat() {
  // Bouml preserved body begin 00041271

    this->ethercatConnectionEstablished = false;
    // Request safe operational state for all slaves
    ec_slave[0].state = EC_STATE_SAFE_OP;

    /* request SAFE_OP state for all slaves */
    ec_writestate(0);

    //stop SOEM, close socket
    ec_close();

    return true;
  // Bouml preserved body end 00041271
}

///stores a ethercat message to the buffer
///@param msgBuffer ethercat message
///@param jointNumber joint number of the sender joint
void EthercatMasterWithThread::setMsgBuffer(const YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber) {
  // Bouml preserved body begin 000414F1

    if (this->automaticSendOn == true) {
      slaveMessages[jointNumber - 1].stctOutput.Set(msgBuffer.stctOutput);
    } else {
      YouBotSlaveMsg localMsg;
      localMsg.stctInput = msgBuffer.stctInput;
      localMsg.stctOutput = msgBuffer.stctOutput;
      localMsg.jointNumber = jointNumber;
      automaticSendOffBufferVector.push_back(localMsg);
    }

  // Bouml preserved body end 000414F1
}

///get a ethercat message form the buffer
///@param msgBuffer ethercat message
///@param jointNumber joint number of the receiver joint
void EthercatMasterWithThread::getMsgBuffer(const unsigned int jointNumber, YouBotSlaveMsg& returnMsg) {
  // Bouml preserved body begin 00041571

    if (this->automaticReceiveOn == true) {
      slaveMessages[jointNumber - 1].stctInput.Get(returnMsg.stctInput);
      slaveMessages[jointNumber - 1].stctOutput.Get(returnMsg.stctOutput);
      slaveMessages[jointNumber - 1].jointNumber.Get(returnMsg.jointNumber);
    } else {
      returnMsg = this->automaticReceiveOffBufferVector[jointNumber - 1];
    }

  // Bouml preserved body end 00041571
}

///stores a mailbox message in a buffer which will be sent to the motor controllers
///@param msgBuffer ethercat mailbox message
///@param jointNumber joint number of the sender joint
void EthercatMasterWithThread::setMailboxMsgBuffer(const YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber) {
  // Bouml preserved body begin 00049D71
    this->mailboxMessages[jointNumber - 1].stctOutput.Set(msgBuffer.stctOutput);
    outstandingMailboxMsgFlag[jointNumber - 1] = true;
    return;
  // Bouml preserved body end 00049D71
}

///gets a mailbox message form the buffer which came form the motor controllers
///@param msgBuffer ethercat mailbox message
///@param jointNumber joint number of the receiver joint
bool EthercatMasterWithThread::getMailboxMsgBuffer(YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber) {
  // Bouml preserved body begin 00049DF1
    if (newInputMailboxMsgFlag[jointNumber - 1] == true) {
      this->mailboxMessages[jointNumber - 1].stctInput.Get(mailboxMsg.stctInput);
      newInputMailboxMsgFlag[jointNumber - 1] = false;
      return true;
    }
    return false;
  // Bouml preserved body end 00049DF1
}

///sends the mailbox Messages which have been stored in the buffer
///@param mailboxMsg ethercat mailbox message
bool EthercatMasterWithThread::sendMailboxMessage(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 00052F71
    //  LOG(trace) << "send mailbox message (buffer two) slave " << mailboxMsg.getSlaveNo();
    mailboxBufferSend[0] = mailboxMsg.stctOutput.moduleAddress;
    mailboxBufferSend[1] = mailboxMsg.stctOutput.commandNumber;
    mailboxBufferSend[2] = mailboxMsg.stctOutput.typeNumber;
    mailboxBufferSend[3] = mailboxMsg.stctOutput.motorNumber;
    mailboxBufferSend[4] = mailboxMsg.stctOutput.value >> 24;
    mailboxBufferSend[5] = mailboxMsg.stctOutput.value >> 16;
    mailboxBufferSend[6] = mailboxMsg.stctOutput.value >> 8;
    mailboxBufferSend[7] = mailboxMsg.stctOutput.value & 0xff;
    if (ec_mbxsend(mailboxMsg.slaveNumber, &mailboxBufferSend, mailboxTimeout)) {
      return true;
    } else {
      return false;
    }
  // Bouml preserved body end 00052F71
}

///receives mailbox messages and stores them in the buffer
///@param mailboxMsg ethercat mailbox message
bool EthercatMasterWithThread::receiveMailboxMessage(YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 00052FF1
    if (ec_mbxreceive(mailboxMsg.slaveNumber, &mailboxBufferReceive, mailboxTimeout)) {
      //    LOG(trace) << "received mailbox message (buffer two) slave " << mailboxMsg.getSlaveNo();
      mailboxMsg.stctInput.replyAddress = (int) mailboxBufferReceive[0];
      mailboxMsg.stctInput.moduleAddress = (int) mailboxBufferReceive[1];
      mailboxMsg.stctInput.status = (int) mailboxBufferReceive[2];
      mailboxMsg.stctInput.commandNumber = (int) mailboxBufferReceive[3];
      mailboxMsg.stctInput.value = (mailboxBufferReceive[4] << 24 | mailboxBufferReceive[5] << 16 | mailboxBufferReceive[6] << 8 | mailboxBufferReceive[7]);
      return true;
    }
    return false;
  // Bouml preserved body end 00052FF1
}

///sends and receives ethercat messages and mailbox messages to and from the motor controllers
///this method is executed in a separate thread
void EthercatMasterWithThread::updateSensorActorValues() {
  // Bouml preserved body begin 0003F771

    long timeToWait = 0;
    boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration pastTime;
    //  int counter = 0;
    boost::posix_time::time_duration realperiode;
    SlaveMessageOutput trajectoryContollerOutput;
    YouBotSlaveMailboxMsg tempMsg;


    while (!stopThread) {

      pastTime = boost::posix_time::microsec_clock::local_time() - startTime;
      timeToWait = timeTillNextEthercatUpdate - pastTime.total_microseconds();

      if (timeToWait < 0 || timeToWait > (int) timeTillNextEthercatUpdate) {
        //    printf("Missed communication period of %d  microseconds it have been %d microseconds \n",timeTillNextEthercatUpdate, (int)pastTime.total_microseconds()+ 100);
      } else {
        boost::this_thread::sleep(boost::posix_time::microseconds(timeToWait));
      }

      // realperiode = boost::posix_time::microsec_clock::local_time() - startTime;
      startTime = boost::posix_time::microsec_clock::local_time();

      /*
            counter++;
            timeSum  = timeSum + realperiode;

            if(counter == 1000){

              double dtotaltime = (double)timeSum.total_microseconds()/counter;
              printf("TotalTime %7.0lf us\n", dtotaltime);
              counter = 0;
              timeSum = startTime - startTime;
            }
       */




      //send and receive data from ethercat
      if (ec_send_processdata() == 0) {
        LOG(warning) << "Sending process data failed";
      }

      if (ec_receive_processdata(this->ethercatTimeout) == 0) {
        if (communicationErrors == 0) {
          LOG(warning) << "Receiving data failed";
        }
        communicationErrors++;
      } else {
        communicationErrors = 0;
      }

      if (communicationErrors > maxCommunicationErrors) {
        LOG(error) << "Lost EtherCAT connection";
        this->closeEthercat();
        stopThread = true;
        break;
      }

      if (ec_iserror())
        LOG(warning) << "there is an error in the soem driver";


      for (unsigned int i = 0; i < nrOfSlaves; i++) {

        //send data
        if(automaticSendOn == true)
          slaveMessages[i].stctOutput.Get(*(ethercatOutputBufferVector[i]));

        //receive data
        if(automaticReceiveOn == true)
          slaveMessages[i].stctInput.Set(*(ethercatInputBufferVector[i]));


        // Limit checker
        if (jointLimitMonitors[i] != NULL) {
          this->jointLimitMonitors[i]->checkLimitsProcessData(*(ethercatInputBufferVector[i]), *(ethercatOutputBufferVector[i]));
        }
        // this->parseYouBotErrorFlags(secondBufferVector[i]);

        //send mailbox messages from first buffer
        if (outstandingMailboxMsgFlag[i]) { 
          this->mailboxMessages[i].stctOutput.Get(tempMsg.stctOutput);
          this->mailboxMessages[i].slaveNumber.Get(tempMsg.slaveNumber);
          sendMailboxMessage(tempMsg);
          outstandingMailboxMsgFlag[i] = false;
          pendingMailboxMsgsReply[i] = true;
        }

        //receive mailbox messages to first buffer
        if (pendingMailboxMsgsReply[i]) {
          this->mailboxMessages[i].slaveNumber.Get(tempMsg.slaveNumber);
          if (receiveMailboxMessage(tempMsg)) {
            this->mailboxMessages[i].stctInput.Set(tempMsg.stctInput);
            newInputMailboxMsgFlag[i] = true;
            pendingMailboxMsgsReply[i] = false;
          }
        }
      }
      
      // Trajectory Controller
      {
        boost::mutex::scoped_lock trajectoryControllerMutex(trajectoryControllerVectorMutex);
        for (unsigned int i = 0; i < nrOfSlaves; i++) {
          if (this->trajectoryControllers[i] != NULL) {
            if (this->trajectoryControllers[i]->updateTrajectoryController(*(ethercatInputBufferVector[i]), trajectoryContollerOutput)) {
              //   printf("send vel slave: %d", i);
              (*(ethercatOutputBufferVector[i])).controllerMode = trajectoryContollerOutput.controllerMode;
              (*(ethercatOutputBufferVector[i])).value = trajectoryContollerOutput.value;
              //copy back
              slaveMessages[i].stctOutput.Set(*(ethercatOutputBufferVector[i]));
            }
          }
        }
      }
      // update Data Traces
      for (unsigned int i = 0; i < nrOfSlaves; i++) {
        {
          boost::mutex::scoped_lock datatraceM(dataTracesMutex);
          if (dataTraces[i] != NULL) {
            ((DataTrace*)dataTraces[i])->updateTrace();
          }
        }
      }
    }
  // Bouml preserved body end 0003F771
}

void EthercatMasterWithThread::parseYouBotErrorFlags(const YouBotSlaveMsg& messageBuffer) {
  // Bouml preserved body begin 000A9E71
    std::stringstream errorMessageStream;
    errorMessageStream << " ";
    std::string errorMessage;
    errorMessage = errorMessageStream.str();


    if (messageBuffer.stctInput.errorFlags & OVER_CURRENT) {
      LOG(error) << errorMessage << "got over current";
      //    throw JointErrorException(errorMessage + "got over current");
    }

    if (messageBuffer.stctInput.errorFlags & UNDER_VOLTAGE) {
      LOG(error) << errorMessage << "got under voltage";
      //    throw JointErrorException(errorMessage + "got under voltage");
    }

    if (messageBuffer.stctInput.errorFlags & OVER_VOLTAGE) {
      LOG(error) << errorMessage << "got over voltage";
      //   throw JointErrorException(errorMessage + "got over voltage");
    }

    if (messageBuffer.stctInput.errorFlags & OVER_TEMPERATURE) {
      LOG(error) << errorMessage << "got over temperature";
      //   throw JointErrorException(errorMessage + "got over temperature");
    }

    if (messageBuffer.stctInput.errorFlags & MOTOR_HALTED) {
      //   LOG(info) << errorMessage << "is halted";
      //   throw JointErrorException(errorMessage + "is halted");
    }

    if (messageBuffer.stctInput.errorFlags & HALL_SENSOR_ERROR) {
      LOG(error) << errorMessage << "got hall sensor problem";
      //   throw JointErrorException(errorMessage + "got hall sensor problem");
    }

    //    if (messageBuffer.stctInput.errorFlags & ENCODER_ERROR) {
    //      LOG(error) << errorMessage << "got encoder problem";
    //      //   throw JointErrorException(errorMessage + "got encoder problem");
    //    }
    //
    //     if (messageBuffer.stctInput.errorFlags & INITIALIZATION_ERROR) {
    //      LOG(error) << errorMessage << "got inizialization problem";
    //      //   throw JointErrorException(errorMessage + "got motor winding problem");
    //    }

//    if (messageBuffer.stctInput.errorFlags & PWM_MODE_ACTIVE) {
    //  LOG(error) << errorMessage << "has PWM mode active";
      //   throw JointErrorException(errorMessage + "the cycle time is violated");
//    }

    if (messageBuffer.stctInput.errorFlags & VELOCITY_MODE) {
      //   LOG(info) << errorMessage << "has velocity mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (messageBuffer.stctInput.errorFlags & POSITION_MODE) {
      //   LOG(info) << errorMessage << "has position mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (messageBuffer.stctInput.errorFlags & TORQUE_MODE) {
      //   LOG(info) << errorMessage << "has torque mode active";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    //    if (messageBuffer.stctInput.errorFlags & EMERGENCY_STOP) {
    //      LOG(info) << errorMessage << "has emergency stop active";
    //      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    //    }
    //
    //    if (messageBuffer.stctInput.errorFlags & FREERUNNING) {
    //   //   LOG(info) << errorMessage << "has freerunning active";
    //      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    //    }

    if (messageBuffer.stctInput.errorFlags & POSITION_REACHED) {
      //    LOG(info) << errorMessage << "has position reached";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (messageBuffer.stctInput.errorFlags & INITIALIZED) {
      //  LOG(info) << errorMessage << "is initialized";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (messageBuffer.stctInput.errorFlags & TIMEOUT) {
      LOG(error) << errorMessage << "has a timeout";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

    if (messageBuffer.stctInput.errorFlags & I2T_EXCEEDED) {
      LOG(error) << errorMessage << "exceeded I2t";
      //   throw JointErrorException(errorMessage + "need to initialize the sinus commutation");
    }

  // Bouml preserved body end 000A9E71
}

std::string EthercatMasterWithThread::configFileName;

std::string EthercatMasterWithThread::configFilepath;


} // namespace youbot
