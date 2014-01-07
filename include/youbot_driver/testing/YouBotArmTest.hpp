#ifndef YOU_BOT_ARM_TEST_H
#define YOU_BOT_ARM_TEST_H

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
class YouBotArmTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(YouBotArmTest);
	CPPUNIT_TEST(youBotArmTest);
	CPPUNIT_TEST_SUITE_END();

public:
	YouBotArmTest();
	virtual ~YouBotArmTest();

	void setUp();
	void tearDown();


	void youBotArmTest();

private:
	unsigned int overallTime;
	unsigned int startTime;
	unsigned int updateCycle;
  const int dof;
};

#endif //YOU_BOT_ARM_TEST_H
