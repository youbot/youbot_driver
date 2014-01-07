#ifndef YOU_BOT_BASE_KINEMATICS_TEST_H
#define YOU_BOT_BASE_KINEMATICS_TEST_H

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
/// A unit test for the base kinematics
///////////////////////////////////////////////////////////////////////////////
class YouBotBaseKinematicsTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(YouBotBaseKinematicsTest);
	CPPUNIT_TEST(youBotBaseKinematicsTest);
	CPPUNIT_TEST_SUITE_END();

public:
	YouBotBaseKinematicsTest();
	virtual ~YouBotBaseKinematicsTest();

	void setUp();
	void tearDown();


	void youBotBaseKinematicsTest();

private:
	unsigned int overallTime;
	unsigned int startTime;
	unsigned int updateCycle;
};

#endif //YOU_BOT_BASE_KINEMATICS_TEST_H
