#include <iostream>
#include <vector>
#include <signal.h>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbot/DataTrace.hpp"

using namespace std;
using namespace youbot;

bool running = true;

void sigintHandler(int signal) {
  running = false;
  std::cout << std::endl << " Interrupt!" << std::endl;
}

int main() {

  signal(SIGINT, sigintHandler);

  try {

    YouBotBase myBase("youbot-base");

    myBase.doJointCommutation();

    DataTrace myTrace(myBase.getBaseJoint(4), "testName");

    JointCurrentSetpoint currentSetpoint;
    currentSetpoint.current = 0.4 * ampere;

    myTrace.startTrace();

    while (running) {
      SLEEP_MILLISEC(1);
      myBase.getBaseJoint(4).setData(currentSetpoint);
      myTrace.updateTrace(currentSetpoint);
    }

    myTrace.stopTrace();
    myTrace.plotTrace();

    EthercatMaster::getInstance().destroy();

  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  } catch (...) {
    std::cout << "unhandled exception" << std::endl;
  }
  return 0;
}
