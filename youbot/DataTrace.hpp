#ifndef YOUBOT_DATATRACE_H
#define YOUBOT_DATATRACE_H

/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public 
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/
#include <vector>
#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <stdlib.h>
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/filesystem.hpp"
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/ConfigFile.hpp"
#include "generic/Exceptions.hpp"
#include "youbot/YouBotJoint.hpp"
#include "youbot/YouBotJointParameter.hpp"
#ifdef ETHERCAT_MASTER_WITHOUT_THREAD
  #include "youbot/EthercatMasterWithoutThread.hpp"
#else
  #include "youbot/EthercatMaster.hpp"
#endif
using namespace boost::posix_time;
namespace youbot {


enum DataTraceCntrollerMode {
    POSITION_CONTROL_RAD,
    POSITION_CONTROL_ENC,
    VELOCITY_CONTROL_RAD_SEC,
    VELOCITY_CONTROL_RPM,
    PWM_CONTROL_MODE,
    CURRENT_CONTROL_MODE,
    TORQUE_CONTROL_MODE
};
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
class DataTrace {
  public:
    DataTrace(YouBotJoint& youBotJoint, const std::string Name);

    virtual ~DataTrace();

    void startTrace();

    void stopTrace();

    void plotTrace();

    void updateTrace(const JointAngleSetpoint& setpoint);

    void updateTrace(const JointVelocitySetpoint& setpoint);

    void updateTrace(const JointRoundsPerMinuteSetpoint& setpoint);

    void updateTrace(const JointCurrentSetpoint& setpoint);

    void updateTrace(const JointTorqueSetpoint& setpoint);

    void updateTrace(const JointPWMSetpoint& setpoint);

    void updateTrace(const JointEncoderSetpoint& setpoint);

    unsigned long getTimeDurationMilliSec();


  private:
    void update();

    YouBotJoint& joint;

    JointSensedAngle sensedAngle;

    JointSensedEncoderTicks sensedEncoderTicks;

    JointSensedVelocity sensedVelocity;

    JointSensedRoundsPerMinute sensedRoundsPerMinute;

    JointSensedCurrent sensedCurrent;

    JointSensedTorque sensedTorque;

    std::fstream file;

    JointAngleSetpoint angleSetpoint;

    JointVelocitySetpoint velocitySetpoint;

    JointRoundsPerMinuteSetpoint roundsPerMinuteSetpoint;

    JointCurrentSetpoint currentSetpoint;

    JointTorqueSetpoint torqueSetpoint;

    JointPWMSetpoint PWMSetpoint;

    JointEncoderSetpoint encoderSetpoint;

    std::fstream parametersBeginTraceFile;

    std::fstream parametersEndTraceFile;

    ptime traceStartTime;

    time_duration timeDuration;

    unsigned long timeDurationMicroSec;

    DataTraceCntrollerMode controllerMode;

    JointSensedPWM actualPWM;

    std::vector<YouBotJointParameterReadOnly*> parameterVector;

    std::string name;

    std::string path;

};

} // namespace youbot
#endif
