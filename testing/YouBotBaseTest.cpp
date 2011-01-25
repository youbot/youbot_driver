#include "YouBotBaseTest.hpp"

using namespace youbot;

YouBotBaseTest::YouBotBaseTest()
{

}

YouBotBaseTest::~YouBotBaseTest()
{

}

void YouBotBaseTest::setUp()
{

}

void YouBotBaseTest::tearDown()
{

}

void YouBotBaseTest::youBotBaseTest_GetBaseJoint()
{
   YouBotBase youBotBase( "youbot-base" );

   CPPUNIT_ASSERT_THROW   ( YouBotBase youbotBase("youbot-base"), std::invalid_argument );

   CPPUNIT_ASSERT_NO_THROW( youBotBase.getBaseJoint(1) );
   CPPUNIT_ASSERT_NO_THROW( youBotBase.getBaseJoint(2) );
   CPPUNIT_ASSERT_NO_THROW( youBotBase.getBaseJoint(3) );
   CPPUNIT_ASSERT_NO_THROW( youBotBase.getBaseJoint(4) );

   CPPUNIT_ASSERT_THROW   ( youBotBase.getBaseJoint(0), std::out_of_range );
   CPPUNIT_ASSERT_THROW   ( youBotBase.getBaseJoint(5), std::out_of_range );
}

void YouBotBaseTest::youBotBaseTest_GetAndSetBaseVelocity()
{
   YouBotBase youBotBase( "youbot-base" );
   
   quantity<si::velocity>         longVel  = 0 * meter_per_second;
   quantity<si::velocity>         transVel = 0 * meter_per_second;
   quantity<si::angular_velocity> angVel   = 0 * radian_per_second;

   CPPUNIT_ASSERT_NO_THROW( youBotBase.setBaseVelocity( longVel, transVel, angVel ));
   
   quantity<si::velocity>         longVelRet;  
   quantity<si::velocity>         transVelRet;
   quantity<si::angular_velocity> angVelRet;
   
   CPPUNIT_ASSERT_NO_THROW( youBotBase.getBaseVelocity( longVelRet, transVelRet, angVelRet ));
   
   CPPUNIT_ASSERT_EQUAL( longVel, longVelRet   );
   CPPUNIT_ASSERT_EQUAL( transVel, transVelRet );
   CPPUNIT_ASSERT_EQUAL( angVel, angVelRet     );
}

void YouBotBaseTest::youBotBaseTest_FourSwedishWheelOmniBaseKinematic()
{
  YouBotBase youBotBase( "youbot-base" );
  FourSwedishWheelOmniBaseKinematicConfiguration conf;
  youBotBase.youBotBaseKinematic.getConfiguration( conf );
  
  quantity<si::length> val  = 0 ;
  CPPUNIT_ASSERT( conf.wheelRadius != val );
  CPPUNIT_ASSERT( conf.lengthBetweenFrontWheels != val );
  CPPUNIT_ASSERT( conf.lengthBetweenFrontAndRearWheels != val );
  CPPUNIT_ASSERT( conf.slideRatio != 0.0 );
  CPPUNIT_ASSERT( conf.rotationRatio != 0.0 );

}

