#include "youbot_driver/testing/YouBotGripperTest.hpp"

using namespace youbot;

YouBotGripperTest::YouBotGripperTest() : dof(5) {

  EthercatMaster::getInstance("youbot-ethercat.cfg", CONFIG_FOLDER_PATH, true);

}

YouBotGripperTest::~YouBotGripperTest() {

}

void YouBotGripperTest::setUp() {
  Logger::logginLevel = trace;
  updateCycle = 2000;

}

void YouBotGripperTest::tearDown() {
  //	EthercatMaster::destroy();
}

void YouBotGripperTest::youBotGripperTest() {

  LOG(info) << __func__ << "\n";
  YouBotManipulator myArm("youbot-manipulator");

  GripperDataTrace myTrace(myArm.getArmGripper().getGripperBar2(), __func__, true);

  TargetPositionReached bar1TargetReched;
  TargetPositionReached bar2TargetReched;
  bool targetReachedBar1 = false;
  bool targetReachedBar2 = false;

  myArm.calibrateGripper(true);
  myTrace.startTrace("Load", "");
  
  //open gripper
  myArm.getArmGripper().open();

  for (int i = 0; i < 40; i++) {
    myArm.getArmGripper().getGripperBar1().getConfigurationParameter(bar1TargetReched);
    bar1TargetReched.getParameter(targetReachedBar1);
    myArm.getArmGripper().getGripperBar2().getConfigurationParameter(bar2TargetReched);
    bar2TargetReched.getParameter(targetReachedBar2);
    myTrace.updateTrace((double)targetReachedBar1);
    if (targetReachedBar1 && targetReachedBar2) {
      break;
    }
  }
  targetReachedBar1 = false;
  targetReachedBar2 = false;

  //close gripper
  myArm.getArmGripper().close();

  for (int i = 0; i < 40; i++) {
    myArm.getArmGripper().getGripperBar1().getConfigurationParameter(bar1TargetReched);
    bar1TargetReched.getParameter(targetReachedBar1);
    myArm.getArmGripper().getGripperBar2().getConfigurationParameter(bar2TargetReched);
    bar2TargetReched.getParameter(targetReachedBar2);
    myTrace.updateTrace((double)targetReachedBar2);
    if (targetReachedBar1 && targetReachedBar2) {
      break;
    }
  }

  myTrace.stopTrace();
  myTrace.plotTrace();


}
