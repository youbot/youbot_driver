#include "youbot_driver/testing/YouBotBaseTestWithoutThread.hpp"

using namespace youbot;

YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread() {
	

}

YouBotBaseTestWithoutThread::~YouBotBaseTestWithoutThread() {
}

void YouBotBaseTestWithoutThread::setUp() {

  Logger::logginLevel = trace;
	ethercatMaster = &EthercatMaster::getInstance("youbot-ethercat.cfg", CONFIG_FOLDER_PATH, false);
	if(ethercatMaster->isThreadActive()){
		LOG(error) << "Thread Active";
		EthercatMaster::destroy();
		ethercatMaster = &EthercatMaster::getInstance("youbot-ethercat.cfg", CONFIG_FOLDER_PATH, false);
	}
	
	jointNO = 4;
	stepStartTime = 1000;
	durationNull = 1000;
	overallTime = 0;
	startTime = 0;
	updateCycle = 2000;
	setAngle.angle = 0 * radian;
	setVel.angularVelocity = 0 * radian_per_second;
	currentSetpoint.current = 0 * ampere;
}

void YouBotBaseTestWithoutThread::tearDown() {

	EthercatMaster::destroy();
}

void YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread_PositionMode() {
	LOG(info) <<__func__<< "\n";
	YouBotBase myBase("youbot-base");
	myBase.doJointCommutation();
	DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
	myBase.getBaseJoint(jointNO).setEncoderToZero();
	if (!ethercatMaster->isThreadActive()) {
		ethercatMaster->sendProcessData();
		ethercatMaster->receiveProcessData();
	}
	myTrace.startTrace();
  
  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myBase.getBaseJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(4).setConfigurationParameter(clearTimeoutFlag);
	
	startTime = myTrace.getTimeDurationMilliSec();
	overallTime = startTime + durationNull + stepStartTime + durationNull;

	while (myTrace.getTimeDurationMilliSec() < overallTime) {
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull) {
			setAngle.angle = 0 * radian;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
						&& myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime) {
			setAngle.angle = 2 * radian;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime) {
			setAngle.angle = 0 * radian;
		}

		myBase.getBaseJoint(jointNO).setData(setAngle);
		if (!ethercatMaster->isThreadActive()) {
			ethercatMaster->sendProcessData();
			ethercatMaster->receiveProcessData();
		}
		myTrace.updateTrace(setAngle);

		SLEEP_MICROSEC(updateCycle);
	}
	myTrace.stopTrace();
	myTrace.plotTrace();
}

void YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread_VelocityMode() {
	LOG(info) <<__func__<< "\n";
	YouBotBase myBase("youbot-base");
	myBase.doJointCommutation();
	DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
	myBase.getBaseJoint(jointNO).setEncoderToZero();
	if (!ethercatMaster->isThreadActive()) {
		ethercatMaster->sendProcessData();
		ethercatMaster->receiveProcessData();
	}
	myTrace.startTrace();
  
  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myBase.getBaseJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(4).setConfigurationParameter(clearTimeoutFlag);
	
	startTime = myTrace.getTimeDurationMilliSec();
	overallTime = startTime + durationNull + stepStartTime + durationNull;

	while (myTrace.getTimeDurationMilliSec() < overallTime) {
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull) {
			setVel.angularVelocity = 0 * radian_per_second;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
						&& myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime) {
			setVel.angularVelocity = 2 * radian_per_second;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime) {
			setVel.angularVelocity = 0 * radian_per_second;
		}

		myBase.getBaseJoint(jointNO).setData(setVel);
		if (!ethercatMaster->isThreadActive()) {
			ethercatMaster->sendProcessData();
			ethercatMaster->receiveProcessData();
		}
		myTrace.updateTrace(setVel);

		SLEEP_MICROSEC(updateCycle);
	}
	myTrace.stopTrace();
	myTrace.plotTrace();
}

void YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread_CurrentMode() {
	LOG(info) <<__func__<< "\n";
	YouBotBase myBase("youbot-base");
	myBase.doJointCommutation();
	DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
	myBase.getBaseJoint(jointNO).setEncoderToZero();
	if (!ethercatMaster->isThreadActive()) {
		ethercatMaster->sendProcessData();
		ethercatMaster->receiveProcessData();
	}
	myTrace.startTrace();
  
  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myBase.getBaseJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(4).setConfigurationParameter(clearTimeoutFlag);
	
	startTime = myTrace.getTimeDurationMilliSec();
	overallTime = startTime + durationNull + stepStartTime + durationNull;

	while (myTrace.getTimeDurationMilliSec() < overallTime) {
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull) {
			currentSetpoint.current = 0 * ampere;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
						&& myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime) {
			currentSetpoint.current = 0.5 * ampere;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime) {
			currentSetpoint.current = 0 * ampere;
		}

		myBase.getBaseJoint(jointNO).setData(currentSetpoint);
		if (!ethercatMaster->isThreadActive()) {
			ethercatMaster->sendProcessData();
			ethercatMaster->receiveProcessData();
		}
		myTrace.updateTrace(currentSetpoint);

		SLEEP_MICROSEC(updateCycle);
	}
	myTrace.stopTrace();
	myTrace.plotTrace();
}
