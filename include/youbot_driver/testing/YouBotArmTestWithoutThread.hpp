#ifndef YouBotArmTestWithoutThread_HPP
#define YouBotArmTestWithoutThread_HPP

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <iostream>
#include <vector>
#include <signal.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

using namespace youbot;

///////////////////////////////////////////////////////////////////////////////
/// A unit test for the youBot arm
///////////////////////////////////////////////////////////////////////////////
class YouBotArmTestWithoutThread : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(YouBotArmTestWithoutThread);
	CPPUNIT_TEST(youBotArmTest);
	CPPUNIT_TEST_SUITE_END();

public:
	YouBotArmTestWithoutThread();
	virtual ~YouBotArmTestWithoutThread();

	void setUp();
	void tearDown();


	void youBotArmTest();

private:
	unsigned int overallTime;
	unsigned int startTime;
	unsigned int updateCycle;
  const int dof;
  EthercatMasterInterface* ethercatMaster;
};

#endif //YouBotArmTestWithoutThread_H
