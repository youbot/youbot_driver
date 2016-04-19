#include "youbot_driver/testing/YouBotBaseKinematicsTest.hpp"

using namespace youbot;

YouBotBaseKinematicsTest::YouBotBaseKinematicsTest() {

  EthercatMaster::getInstance("youbot-ethercat.cfg", CONFIG_FOLDER_PATH, true);


}

YouBotBaseKinematicsTest::~YouBotBaseKinematicsTest() {

}

void YouBotBaseKinematicsTest::setUp() {
  Logger::logginLevel = trace;
  updateCycle = 2000;

}

void YouBotBaseKinematicsTest::tearDown() {
  //	EthercatMaster::destroy();
}

void YouBotBaseKinematicsTest::youBotBaseKinematicsTest() {

  LOG(info) << __func__ << "\n";
  YouBotBase myBase("youbot-base");
  myBase.doJointCommutation();
  std::stringstream jointNameStream;
  boost::ptr_vector<DataTrace> myTrace;
  unsigned int step1 = 1000;
  unsigned int step2 = 4000;
  unsigned int step3 = 7000;
  unsigned int step4 = 10000;
  unsigned int step5 = 13000;
  unsigned int step6 = 16000;

  // create variables to set and get the base cartesian velocity and pose
  quantity<si::velocity> longitudinalVelocity = 0.0 * meter_per_second;
  quantity<si::velocity> transversalVelocity = 0.0 * meter_per_second;
  quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;

  for (int i = 1; i <= 4; i++) {
    jointNameStream << "Joint_" << i << "_" << __func__;
    myTrace.push_back(new DataTrace(myBase.getBaseJoint(i), jointNameStream.str(), true));
    jointNameStream.str("");
    myBase.getBaseJoint(i).setEncoderToZero();
  }

  for (int i = 0; i < 4; i++) {
    myTrace[i].startTrace();
  }

  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + step6 +10;

  while (myTrace[0].getTimeDurationMilliSec() < overallTime) {
    if (myTrace[0].getTimeDurationMilliSec() > startTime + step1) {
      longitudinalVelocity = 0.0 * meter_per_second;
      transversalVelocity = 0.0 * meter_per_second;
      angularVelocity = 0 * radian_per_second;
    }
    if (myTrace[0].getTimeDurationMilliSec() > startTime + step1
            && myTrace[0].getTimeDurationMilliSec() < startTime + step2) {
      longitudinalVelocity = 0.2 * meter_per_second;
      transversalVelocity = 0.0 * meter_per_second;
      angularVelocity = 0 * radian_per_second;
    }
    if (myTrace[0].getTimeDurationMilliSec() > startTime + step2
            && myTrace[0].getTimeDurationMilliSec() < startTime + step3) {
      longitudinalVelocity = 0.0 * meter_per_second;
      transversalVelocity = 0.2 * meter_per_second;
      angularVelocity = 0.0 * radian_per_second;
    }
    
        if (myTrace[0].getTimeDurationMilliSec() > startTime + step3
            && myTrace[0].getTimeDurationMilliSec() < startTime + step4) {
      longitudinalVelocity = 0.0 * meter_per_second;
      transversalVelocity = 0.0 * meter_per_second;
      angularVelocity = 0.2 * radian_per_second;
    }
    
        if (myTrace[0].getTimeDurationMilliSec() > startTime + step4
            && myTrace[0].getTimeDurationMilliSec() < startTime + step5) {
      longitudinalVelocity = 0.1 * meter_per_second;
      transversalVelocity = 0.1 * meter_per_second;
      angularVelocity = 0.0 * radian_per_second;
    }
    
    if (myTrace[0].getTimeDurationMilliSec() > startTime + step5
            && myTrace[0].getTimeDurationMilliSec() < startTime + step6) {
      longitudinalVelocity = 0.1 * meter_per_second;
      transversalVelocity = 0.1 * meter_per_second;
      angularVelocity = 0.0 * radian_per_second;
    }
    
    if (myTrace[0].getTimeDurationMilliSec() > startTime + step6) {
      longitudinalVelocity = 0 * meter_per_second;
      transversalVelocity = 0 * meter_per_second;
      angularVelocity = 0 * radian_per_second;
    }

    myBase.setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
    for (int i = 0; i < 4; i++) {
      myTrace[i].updateTrace();
    }


    SLEEP_MICROSEC(updateCycle);
  }
  for (int i = 0; i < 4; i++) {
    myTrace[i].stopTrace();
    myTrace[i].plotTrace();
  }

}
