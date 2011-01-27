#ifndef YOUBOT_MOTORPROTECTION_H
#define YOUBOT_MOTORPROTECTION_H


#include "boost/date_time/posix_time/posix_time.hpp"
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/Exceptions.hpp"
#include "youbot/ProtocolDefinitions.hpp"
#include "youbot/YouBotSlaveMsg.hpp"

using namespace boost::posix_time;
namespace youbot {

struct CurrentWithTimeSlice {
    quantity<si::current> current;

    quantity<si::time> timeSlice;

};
class MotorProtection {
  public:
    MotorProtection(const quantity<si::current>& maxContinuousCurrent, const quantity<si::time>& thermalTimeConstantWinding);

    virtual ~MotorProtection();

    bool isRMSCurrentOverLimit(const quantity<si::current>& actualCurrent, const ptime& timestamp);

    bool createSafeMotorCommands(YouBotSlaveMsg& MotorCommand);


  private:
    std::deque<CurrentWithTimeSlice> monitoredMeasurements;

    quantity<si::time> maxWindowTime;

    quantity<si::time> summedTime;

    quantity<si::current> RMSCurrent;

    quantity<si::current> allowedContinuousCurrent;

    ptime lastTimestamp;

};

} // namespace youbot
#endif
