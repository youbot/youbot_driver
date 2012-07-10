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
#include "youbot/JointTrajectoryController.hpp"
namespace youbot {

JointTrajectoryController::JointTrajectoryController() {
  // Bouml preserved body begin 000EA0F1
		this->pid.initPid(1.0, 0, 0, 1000, -1000);
		time = boost::posix_time::microsec_clock::local_time();
		last_time = boost::posix_time::microsec_clock::local_time();

    this->isControllerActive = false;
		this->targetPosition = 0;
		this->targetVelocity = 0;
		this->targetAcceleration = 0;

  // Bouml preserved body end 000EA0F1
}

JointTrajectoryController::~JointTrajectoryController() {
  // Bouml preserved body begin 000EA171
  // Bouml preserved body end 000EA171
}

void JointTrajectoryController::getConfigurationParameter(double& PParameter, double& IParameter, double& DParameter, double& IClippingMax, double& IClippingMin) {
  // Bouml preserved body begin 000ED671
  if(this->isControllerActive)
    throw JointParameterException("The trajectory controller is running");
	this->pid.getGains(PParameter, IParameter, DParameter, IClippingMax, IClippingMin);
  // Bouml preserved body end 000ED671
}

void JointTrajectoryController::setConfigurationParameter(const double PParameter, const double IParameter, const double DParameter, const double IClippingMax, const double IClippingMin) {
  // Bouml preserved body begin 001000F1
  if(this->isControllerActive)
    throw JointParameterException("The trajectory controller is running");
  this->pid.setGains(PParameter, IParameter, DParameter, IClippingMax, IClippingMin);
  // Bouml preserved body end 001000F1
}

void JointTrajectoryController::setTrajectory(const JointTrajectory& input_traj) {
  // Bouml preserved body begin 000E84F1

	if(input_traj.segments.size() == 0)
		throw std::runtime_error("Invalid trajectory");

	boost::posix_time::ptime time_now = boost::posix_time::microsec_clock::local_time();

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &new_traj = *new_traj_ptr;
	
	 // ------ Grabs the trajectory that we're currently following.

  boost::shared_ptr<const SpecifiedTrajectory> prev_traj_ptr;
  current_trajectory_box_.Get(prev_traj_ptr);
  if (!prev_traj_ptr)
  {
    throw std::runtime_error("The current trajectory can never be null");
    return;
  }
  const SpecifiedTrajectory &prev_traj = *prev_traj_ptr;
	
	// ------ Copies over the segments from the previous trajectory that are still useful.

  // Useful segments are still relevant after the current time.
  int first_useful = -1;
  while (first_useful + 1 < (int)prev_traj.size() && prev_traj[first_useful + 1].start_time <= time_now)
  {
    ++first_useful;
  }

  // Useful segments are not going to be completely overwritten by the message's splines.
  int last_useful = -1;
  while (last_useful + 1 < (int)prev_traj.size() && prev_traj[last_useful + 1].start_time < time_now)
  {
    ++last_useful;
  }

  if (last_useful < first_useful)
    first_useful = last_useful;

  // Copies over the old segments that were determined to be useful.
  for (int i = std::max(first_useful,0); i <= last_useful; ++i)
  {
    new_traj.push_back(prev_traj[i]);
  }

  // We always save the last segment so that we know where to stop if
  // there are no new segments.
  if (new_traj.size() == 0)
    new_traj.push_back(prev_traj[prev_traj.size() - 1]);

  // ------ Determines when and where the new segments start

  // Finds the end conditions of the final segment
  Segment &last = new_traj[new_traj.size() - 1];
	double prev_positions;
  double prev_velocities;
  double prev_accelerations;

  LOG(debug) << "Initial conditions for new set of splines:";

  sampleSplineWithTimeBounds(last.spline.coef, last.duration.fractional_seconds(),
                               last.start_time.time_of_day().fractional_seconds(),
                               prev_positions, prev_velocities, prev_accelerations);
  //  ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i],
  //            prev_accelerations[i], joints_[i]->joint_->name.c_str());
  

  // ------ Tacks on the new segments
	
	
	double positions;
  double velocities;
  double accelerations;
	


  std::vector<boost::posix_time::time_duration> durations;
  durations[0] = input_traj.segments[0].time_from_start;
	
  for (size_t i = 1; i < input_traj.segments.size(); ++i)
    durations.push_back(input_traj.segments[i].time_from_start - input_traj.segments[i-1].time_from_start);
	
  for (unsigned int i = 0; i < input_traj.segments.size(); ++i)
  {
    Segment seg;

    seg.start_time = input_traj.start_time + input_traj.segments[i].time_from_start;
    seg.duration = durations[i];


		positions = input_traj.segments[i].positions.value();
		velocities = input_traj.segments[i].velocities.value();
		accelerations = input_traj.segments[i].accelerations.value();
		
    // Converts the boundary conditions to splines.

   //   if (prev_accelerations.size() > 0 && accelerations.size() > 0)
   //   {
        getQuinticSplineCoefficients(
          prev_positions, prev_velocities, prev_accelerations,
          positions, velocities, accelerations,
          durations[i].fractional_seconds(),
          seg.spline.coef);
				/*
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          positions[j], velocities[j],
          durations[i],
          seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    */

    // Pushes the splines onto the end of the new trajectory.

    new_traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }
	
	 // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    throw std::runtime_error("The new trajectory was null!");
    return;
  }

  current_trajectory_box_.Set(new_traj_ptr);
  LOG(debug) << "The new trajectory has " << new_traj.size() << " segments";
  
  // Bouml preserved body end 000E84F1
}

void JointTrajectoryController::cancelCurrentTrajectory() {
  // Bouml preserved body begin 000F0771
   
      LOG(error) << "Could not cancel the trajectory!";

  // Bouml preserved body end 000F0771
}

bool JointTrajectoryController::isTrajectoryControllerActive() {
  // Bouml preserved body begin 000EECF1
  return this->isControllerActive;
  // Bouml preserved body end 000EECF1
}

bool JointTrajectoryController::updateTrajectoryController(const SlaveMessageInput& actual, SlaveMessageOutput& velocity) {
  // Bouml preserved body begin 000EA271


	time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration dt = time - last_time;
  last_time = time;

  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
  current_trajectory_box_.Get(traj_ptr);
  if (!traj_ptr)
    LOG(error) << "The current trajectory can never be null";

  // Only because this is what the code originally looked like.
  const SpecifiedTrajectory &traj = *traj_ptr;
	
	if(traj.size() == 0){
      this->isControllerActive = false;
      return false;
  }

  // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
  int seg = -1;
  while (seg + 1 < (int)traj.size() &&
         traj[seg+1].start_time < time)
  {
    ++seg;
  }

  if (seg == -1)
  {
    if (traj.size() == 0)
      LOG(error) << "No segments in the trajectory";
    else
      LOG(error) << "No earlier segments.";
    return false;
  }

  // ------ Trajectory Sampling

    sampleSplineWithTimeBounds(traj[seg].spline.coef, traj[seg].duration.fractional_seconds(),
                               (time - traj[seg].start_time).fractional_seconds(),
                               targetPosition, targetVelocity, targetAcceleration);


  // ------ Trajectory Following


    double error = actual.actualPosition - targetPosition;
    velocity.value = pid.updatePid(error, actual.actualVelocity - targetVelocity, dt);
  

    velocity.controllerMode = CURRENT_MODE;

    
    this->isControllerActive = true;
    return true;
  // Bouml preserved body end 000EA271
}

void JointTrajectoryController::getLastTargetPosition(JointEncoderSetpoint& position) {
  // Bouml preserved body begin 000EED71

      position.encoderTicks = targetPosition;

  // Bouml preserved body end 000EED71
}

void JointTrajectoryController::getQuinticSplineCoefficients(const double start_pos, const double start_vel, const double start_acc, const double end_pos, const double end_vel, const double end_acc, const double time, std::vector<double>& coefficients) {
  // Bouml preserved body begin 000FFE71
	coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5*end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
  // Bouml preserved body end 000FFE71
}

void JointTrajectoryController::sampleQuinticSpline(const std::vector<double>& coefficients, const double time, double& position, double& velocity, double& acceleration) {
  // Bouml preserved body begin 000FFEF1
	// create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
      t[1]*coefficients[1] +
      t[2]*coefficients[2] +
      t[3]*coefficients[3] +
      t[4]*coefficients[4] +
      t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
      2.0*t[1]*coefficients[2] +
      3.0*t[2]*coefficients[3] +
      4.0*t[3]*coefficients[4] +
      5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
      6.0*t[1]*coefficients[3] +
      12.0*t[2]*coefficients[4] +
      20.0*t[3]*coefficients[5];
  // Bouml preserved body end 000FFEF1
}

void JointTrajectoryController::getCubicSplineCoefficients(const double start_pos, const double start_vel, const double end_pos, const double end_vel, const double time, std::vector<double>& coefficients) {
  // Bouml preserved body begin 000FFF71
	coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
  // Bouml preserved body end 000FFF71
}

void JointTrajectoryController::generatePowers(const int n, const double x, double* powers) {
  // Bouml preserved body begin 000FFFF1
	powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
  // Bouml preserved body end 000FFFF1
}

void JointTrajectoryController::sampleSplineWithTimeBounds(const std::vector<double>& coefficients, const double duration, const double time, double& position, double& velocity, double& acceleration) {
  // Bouml preserved body begin 00100071
	 if (time < 0)
  {
    double _;
    sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    double _;
    sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    sampleQuinticSpline(coefficients, time,
                        position, velocity, acceleration);
  }
  // Bouml preserved body end 00100071
}


} // namespace youbot
