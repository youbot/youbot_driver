#ifndef YOUBOT_JOINTTRAJECTORYCONTROLLER_H
#define YOUBOT_JOINTTRAJECTORYCONTROLLER_H

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
#include <string>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include "youbot_driver/generic/Units.hpp"
#include "youbot_driver/generic/Time.hpp"
#include "youbot_driver/generic/PidController.hpp"
#include "youbot_driver/generic-joint/JointTrajectory.hpp"
#include "youbot_driver/youbot/YouBotSlaveMsg.hpp"
#include "youbot_driver/generic/dataobjectlockfree/DataObjectLockFree.hpp"

namespace youbot {



/// Spline for a joint trajectory 
/// coef[0] + coef[1]*t + ... + coef[5]*t^5
struct Spline
{
  std::vector<double> coef;

  Spline() : coef(6, 0.0) {}
};

/// Joint trajectory segment
struct Segment
 {
  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration duration;
  Spline spline;
};

///////////////////////////////////////////////////////////////////////////////
/// Joint Trajectory Controller
///////////////////////////////////////////////////////////////////////////////
class JointTrajectoryController {
  public:
    JointTrajectoryController();

    virtual ~JointTrajectoryController();


  private:
    JointTrajectoryController(const JointTrajectoryController & source);

    JointTrajectoryController & operator=(const JointTrajectoryController & source);


  public:
    void getConfigurationParameter(double& PParameter, double& IParameter, double& DParameter, double& IClippingMax, double& IClippingMin);

    void setConfigurationParameter(const double PParameter, const double IParameter, const double DParameter, const double IClippingMax, const double IClippingMin);

    void setTrajectory(const JointTrajectory& input_traj);

    void cancelCurrentTrajectory();

    bool isTrajectoryControllerActive();

    bool updateTrajectoryController(const SlaveMessageInput& actual, SlaveMessageOutput& velocity);

    void getLastTargetPosition(JointAngleSetpoint& position);
    
    void getLastTargetVelocity(JointVelocitySetpoint& velocity);
    
    void setGearRatio(const double& ratio) {this->gearRatio = ratio;};
    
    void setEncoderTicksPerRound(const int& encoderTicks) {this->encoderTicksPerRound = encoderTicks;};
    
    void setInverseMovementDirection(const bool invDirection) {this->inverseDirection = invDirection;};


  private:
    void getQuinticSplineCoefficients(const double start_pos, const double start_vel, const double start_acc, const double end_pos, const double end_vel, const double end_acc, const double time, std::vector<double>& coefficients);

    void sampleQuinticSpline(const std::vector<double>& coefficients, const double time, double& position, double& velocity, double& acceleration);

    void getCubicSplineCoefficients(const double start_pos, const double start_vel, const double end_pos, const double end_vel, const double time, std::vector<double>& coefficients);

    void generatePowers(const int n, const double x, double* powers);

    void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, const double duration, const double time, double& position, double& velocity, double& acceleration);

    bool isControllerActive;

    PidController pid;

    boost::posix_time::ptime time;

    boost::posix_time::ptime last_time;

    typedef std::vector<Segment> SpecifiedTrajectory;

    DataObjectLockFree< boost::shared_ptr<const SpecifiedTrajectory> > current_trajectory_box_;

    double targetPosition;

    double targetVelocity;

    double targetAcceleration;
    
    int encoderTicksPerRound;
    
    double gearRatio;
    
    bool inverseDirection;
    
    double pose_error;
    
    double velocity_error;
    
    double velsetpoint;
    
    double time_till_seg_start;
    double duration;
    double actualpose;
    double actualvel;

};

} // namespace youbot
#endif
