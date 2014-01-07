#ifndef YOU_BOT_GRIPPER_TEST_H
#define YOU_BOT_GRIPPER_TEST_H

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <iostream>
#include <vector>
#include <signal.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"
#include "youbot_driver/youbot/GripperDataTrace.hpp"

using namespace youbot;

///////////////////////////////////////////////////////////////////////////////
/// A unit test for the youBot gripper
///////////////////////////////////////////////////////////////////////////////
class YouBotGripperTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(YouBotGripperTest);
	CPPUNIT_TEST(youBotGripperTest);
	CPPUNIT_TEST_SUITE_END();

public:
	YouBotGripperTest();
	virtual ~YouBotGripperTest();

	void setUp();
	void tearDown();


	void youBotGripperTest();

private:
	unsigned int overallTime;
	unsigned int startTime;
	unsigned int updateCycle;
  const int dof;
};

#endif //YOU_BOT_GRIPER_TEST_H
