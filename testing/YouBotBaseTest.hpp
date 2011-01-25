#ifndef YOU_BOT_BASE_TEST_H
#define YOU_BOT_BASE_TEST_H

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include "youbot/YouBotBase.hpp"

class YouBotBaseTest : public CppUnit::TestFixture
{
   CPPUNIT_TEST_SUITE    ( YouBotBaseTest                                   );
   CPPUNIT_TEST          ( youBotBaseTest_GetBaseJoint                      );
   CPPUNIT_TEST          ( youBotBaseTest_GetAndSetBaseVelocity             );
   CPPUNIT_TEST          ( youBotBaseTest_FourSwedishWheelOmniBaseKinematic );
   CPPUNIT_TEST_SUITE_END(                                                  );
   
   public:
      YouBotBaseTest( );
      virtual ~YouBotBaseTest( );

      void setUp   ( );
      void tearDown( );

      void youBotBaseTest_GetBaseJoint                     ( );
      void youBotBaseTest_GetAndSetBaseVelocity            ( );
      void youBotBaseTest_FourSwedishWheelOmniBaseKinematic( );
   
   private:
//      youbot::YouBotBase* myYouBotBase;
//      void operator =( const YouBotBaseTest &copy );
};

#endif //YOU_BOT_BASE_TEST_H
