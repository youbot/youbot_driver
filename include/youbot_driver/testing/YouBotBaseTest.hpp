#ifndef YOU_BOT_BASE_TEST_H
#define YOU_BOT_BASE_TEST_H

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <iostream>
#include <vector>
#include <signal.h>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/DataTrace.hpp"

using namespace youbot;

///////////////////////////////////////////////////////////////////////////////
/// A unit test for the youBot base
///////////////////////////////////////////////////////////////////////////////
class YouBotBaseTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(YouBotBaseTest);
	CPPUNIT_TEST(youBotBaseTest_PositionMode);
	CPPUNIT_TEST(youBotBaseTest_VelocityMode);
	CPPUNIT_TEST(youBotBaseTest_CurrentMode);
	CPPUNIT_TEST_SUITE_END();

public:
	YouBotBaseTest();
	virtual ~YouBotBaseTest();

	void setUp();
	void tearDown();


	void youBotBaseTest_PositionMode();
	void youBotBaseTest_VelocityMode();
	void youBotBaseTest_CurrentMode();

private:
	unsigned int jointNO;
	unsigned int stepStartTime;
	unsigned int durationNull;
	unsigned int overallTime;
	unsigned int startTime;
	unsigned int updateCycle;
	JointAngleSetpoint setAngle;
	JointVelocitySetpoint setVel;
	JointCurrentSetpoint currentSetpoint;

};

#endif //YOU_BOT_BASE_TEST_H
