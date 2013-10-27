#include "YouBotBaseTest.hpp"

using namespace youbot;

YouBotBaseTest::YouBotBaseTest() {

	EthercatMaster::getInstance("youbot-ethercat.cfg", "../config/", true);


}

YouBotBaseTest::~YouBotBaseTest() {

}

void YouBotBaseTest::setUp() {
	Logger::logginLevel = trace;
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

void YouBotBaseTest::tearDown() {
//	EthercatMaster::destroy();
}

void YouBotBaseTest::youBotBaseTest_PositionMode() {

	LOG(info) <<__func__<< "\n";
	YouBotBase myBase("youbot-base");
	myBase.doJointCommutation();
	DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
	myBase.getBaseJoint(jointNO).setEncoderToZero();
	myTrace.startTrace();
	
	startTime = myTrace.getTimeDurationMilliSec();
	overallTime = startTime + durationNull + stepStartTime + durationNull;

	while (myTrace.getTimeDurationMilliSec() < overallTime) {
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull) {
			setAngle.angle = 0 * radian;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
						&& myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime) {
			setAngle.angle = 1 * radian;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime) {
			setAngle.angle = 0 * radian;
		}

		myBase.getBaseJoint(jointNO).setData(setAngle);
		myTrace.updateTrace(setAngle);

		SLEEP_MICROSEC(updateCycle);
	}
	myTrace.stopTrace();
	myTrace.plotTrace();
}

void YouBotBaseTest::youBotBaseTest_VelocityMode() {
	LOG(info) <<__func__<< "\n";
	YouBotBase myBase("youbot-base");
	myBase.doJointCommutation();
	DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
	myBase.getBaseJoint(jointNO).setEncoderToZero();
	myTrace.startTrace();
	
	startTime = myTrace.getTimeDurationMilliSec();
	overallTime = startTime + durationNull + stepStartTime + durationNull;

	while (myTrace.getTimeDurationMilliSec() < overallTime) {
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull) {
			setVel.angularVelocity = 0 * radian_per_second;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
						&& myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime) {
			setVel.angularVelocity = 1 * radian_per_second;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime) {
			setVel.angularVelocity = 0 * radian_per_second;
		}

		myBase.getBaseJoint(jointNO).setData(setVel);
		myTrace.updateTrace(setVel);

		SLEEP_MICROSEC(updateCycle);
	}
	myTrace.stopTrace();
	myTrace.plotTrace();
}

void YouBotBaseTest::youBotBaseTest_CurrentMode() {
	LOG(info) <<__func__<< "\n";
	YouBotBase myBase("youbot-base");
	myBase.doJointCommutation();
	DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
	myBase.getBaseJoint(jointNO).setEncoderToZero();
	myTrace.startTrace();
	
	startTime = myTrace.getTimeDurationMilliSec();
	overallTime = startTime + durationNull + stepStartTime + durationNull;

	while (myTrace.getTimeDurationMilliSec() < overallTime) {
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull) {
			currentSetpoint.current = 0 * ampere;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
						&& myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime) {
			currentSetpoint.current = 0.3 * ampere;
		}
		if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime) {
			currentSetpoint.current = 0 * ampere;
		}

		myBase.getBaseJoint(jointNO).setData(currentSetpoint);
		myTrace.updateTrace(currentSetpoint);

		SLEEP_MICROSEC(updateCycle);
	}
	myTrace.stopTrace();
	myTrace.plotTrace();
}