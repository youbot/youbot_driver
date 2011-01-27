
#include "youbot/MotorProtection.hpp"
namespace youbot {

MotorProtection::MotorProtection(const quantity<si::current>& maxContinuousCurrent, const quantity<si::time>& thermalTimeConstantWinding) {
  // Bouml preserved body begin 00087671
    summedTime = 0 * second;
    lastTimestamp = microsec_clock::local_time();
    allowedContinuousCurrent = maxContinuousCurrent;
    maxWindowTime = thermalTimeConstantWinding;

  // Bouml preserved body end 00087671
}

MotorProtection::~MotorProtection() {
  // Bouml preserved body begin 000876F1
  // Bouml preserved body end 000876F1
}

bool MotorProtection::isRMSCurrentOverLimit(const quantity<si::current>& actualCurrent, const ptime& timestamp) {
  // Bouml preserved body begin 00085C71


    time_duration dt;
    dt = timestamp - lastTimestamp;
    lastTimestamp = timestamp;

    quantity<si::time> timeSlice = ((double) dt.total_milliseconds() / 1000.0) * second;
    CurrentWithTimeSlice element;
    element.current = actualCurrent;
    element.timeSlice = timeSlice;

    monitoredMeasurements.push_back(element);
    summedTime = summedTime + timeSlice;


    if (summedTime > maxWindowTime) {
      double i2t = 0;
      for (unsigned int i = 0; i < monitoredMeasurements.size(); i++) {
        i2t = i2t + monitoredMeasurements[i].current.value() * monitoredMeasurements[i].current.value() * monitoredMeasurements[i].timeSlice.value();

      }
      RMSCurrent = sqrt(i2t / summedTime.value()) * ampere;


      while (summedTime > maxWindowTime) {
        summedTime = summedTime - monitoredMeasurements.front().timeSlice;
        monitoredMeasurements.pop_front();
      }
   //   std::cout << " RMS Current: " << RMSCurrent << "\r";

    } else {
      return false;
    }

    if (RMSCurrent > allowedContinuousCurrent) {
      return true;
    }

    return false;

  // Bouml preserved body end 00085C71
}

bool MotorProtection::createSafeMotorCommands(YouBotSlaveMsg& MotorCommand) {
  // Bouml preserved body begin 00085CF1

    if (RMSCurrent > allowedContinuousCurrent) {
      MotorCommand.stctOutput.controllerMode = VELOCITY_CONTROL;
      MotorCommand.stctOutput.positionOrSpeed = 0;
      return true;
    }
    return false;

  // Bouml preserved body end 00085CF1
}


} // namespace youbot
