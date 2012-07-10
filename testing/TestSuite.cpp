#include "YouBotBaseTest.hpp"
#include "YouBotBaseTestWithoutThread.hpp"
#include "YouBotBaseKinematicsTest.hpp"
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>

CPPUNIT_TEST_SUITE_REGISTRATION( YouBotBaseTest );
CPPUNIT_TEST_SUITE_REGISTRATION( YouBotBaseTestWithoutThread );
CPPUNIT_TEST_SUITE_REGISTRATION( YouBotBaseKinematicsTest );

int main(int argc, char* argv[]) {
  std::cout << "Attention! All wheels of the youBot will move during the test. \nThe youBot should not stand on the ground and the wheels should be in the air!" << std::endl;
  char input = 0;

  while (input != 'y' && input != 'n') {
    std::cout << "Are all wheels off the ground? [n/y]" << std::endl;

    input = getchar();

    if (input == 'n') {
      return 0;
    }
  }
  
  CppUnit::Test *suite = CppUnit::TestFactoryRegistry::getRegistry().makeTest();
  CppUnit::TextUi::TestRunner runner;
  runner.addTest( suite );
  runner.setOutputter( new CppUnit::CompilerOutputter( &runner.result(), std::cerr ) );
  /** let the test run */
  bool wasSucessful = runner.run();
  
  /** check whether it was sucessfull or not */
  return wasSucessful ? 0 : 1;
}


