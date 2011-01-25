#ifndef YOUBOT_JOINT_H
#define YOUBOT_JOINT_H


#include <vector>
#include "generic/Units.hpp"
#include "generic-joint/JointData.hpp"
#include "generic-joint/JointParameter.hpp"

namespace youbot {

enum SyncMode {
  NON_BLOCKING

};
class Joint {
  public:
    virtual void setData(const JointDataSetpoint& data, SyncMode communicationMode) = 0;

    virtual void getData(JointData& data) = 0;

    virtual void setConfigurationParameter(const JointParameter& parameter) = 0;

    virtual void getConfigurationParameter(JointParameter& parameter) = 0;

};

} // namespace youbot
#endif
