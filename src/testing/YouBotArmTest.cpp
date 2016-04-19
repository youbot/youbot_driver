#include "youbot_driver/testing/YouBotArmTest.hpp"

using namespace youbot;

YouBotArmTest::YouBotArmTest():dof(5) {

  EthercatMaster::getInstance("youbot-ethercat.cfg", CONFIG_FOLDER_PATH, true);


}

YouBotArmTest::~YouBotArmTest() {

}

void YouBotArmTest::setUp() {
  Logger::logginLevel = trace;
  updateCycle = 2000;

}

void YouBotArmTest::tearDown() {
  //	EthercatMaster::destroy();
}

void YouBotArmTest::youBotArmTest() {

  LOG(info) << __func__ << "\n";
  YouBotManipulator myArm("youbot-manipulator");
  myArm.doJointCommutation();
  myArm.calibrateManipulator();

  std::stringstream jointNameStream;
  boost::ptr_vector<DataTrace> myTrace;
  std::vector<JointAngleSetpoint> upstretchedpose;
  std::vector<JointAngleSetpoint> foldedpose;
  JointAngleSetpoint desiredJointAngle;

  desiredJointAngle.angle = 2.56244 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.04883 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = -2.43523 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.73184 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.5 * radian;
  upstretchedpose.push_back(desiredJointAngle);

  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = -0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.2 * radian;
  foldedpose.push_back(desiredJointAngle);


  for (int i = 1; i <= dof; i++) {
    jointNameStream << "Joint_" << i << "_" << __func__;
    myTrace.push_back(new DataTrace(myArm.getArmJoint(i), jointNameStream.str(), true));
    jointNameStream.str("");
  }


  for (int i = 0; i < dof; i++) {
    myTrace[i].startTrace();
  }

  // 1 sec no movement
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 1000;  
  while (myTrace[0].getTimeDurationMilliSec() < overallTime) {
    for (int i = 0; i < dof; i++) {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }

  // move to upstretched position
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 5000;
  myArm.setJointData(upstretchedpose);
  while (myTrace[0].getTimeDurationMilliSec() < overallTime) {
    for (int i = 0; i < dof; i++) {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }
  
  // move to folded position
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 5000;
  myArm.setJointData(foldedpose);
  while (myTrace[0].getTimeDurationMilliSec() < overallTime) {
    for (int i = 0; i < dof; i++) {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }
  
  // 1 sec no movement
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 1000;  
  while (myTrace[0].getTimeDurationMilliSec() < overallTime) {
    for (int i = 0; i < dof; i++) {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }
  
  for (int i = 0; i < dof; i++) {
    myTrace[i].stopTrace();
    myTrace[i].plotTrace();
  }

}
