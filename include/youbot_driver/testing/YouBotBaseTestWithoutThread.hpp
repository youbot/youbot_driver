#ifndef YouBotBaseTestWithoutThread_HPP
#define YouBotBaseTestWithoutThread_HPP

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
/// A unit test for one youbot joint communicating without a thread
///////////////////////////////////////////////////////////////////////////////
class YouBotBaseTestWithoutThread : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(YouBotBaseTestWithoutThread);
	CPPUNIT_TEST(YouBotBaseTestWithoutThread_PositionMode);
	CPPUNIT_TEST(YouBotBaseTestWithoutThread_VelocityMode);
	CPPUNIT_TEST(YouBotBaseTestWithoutThread_CurrentMode);
	CPPUNIT_TEST_SUITE_END();

public:
	YouBotBaseTestWithoutThread();
	virtual ~YouBotBaseTestWithoutThread();

	void setUp();
	void tearDown();


	void YouBotBaseTestWithoutThread_PositionMode();
	void YouBotBaseTestWithoutThread_VelocityMode();
	void YouBotBaseTestWithoutThread_CurrentMode();

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
	EthercatMasterInterface* ethercatMaster;

};

#endif //YouBotBaseTestWithoutThread_HPP
