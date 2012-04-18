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

JointTrajectoryController::JointTrajectoryController(YouBotJoint& joint) 
: joint(joint){
  // Bouml preserved body begin 000EA0F1
  ethercatMaster = static_cast<EthercatMasterWithThread*>(&(EthercatMaster::getInstance()));
          
  GearRatio gearR;
  joint.getConfigurationParameter(gearR);
  gearR.getParameter(gearRatio);
  
  // Bouml preserved body end 000EA0F1
}

JointTrajectoryController::~JointTrajectoryController() {
  // Bouml preserved body begin 000EA171
  // Bouml preserved body end 000EA171
}

/// calculates all trajectory values for the future and sets all the the ethercat master
/// if the trajectory is still active the the values in the next buffer
void JointTrajectoryController::setTrajectory(const std::vector< quantity<plane_angle> >& positions, const std::vector< quantity<angular_velocity> >& velocities, const std::vector< quantity<angular_acceleration> >& accelerations) {
  // Bouml preserved body begin 000E84F1
  //check data
  // vel != 0
  // acc != 0
  // position in limits
  //if(velocity == 0 *radian_per_second)
  //  return;
  JointSensedAngle currentPosition;
  quantity<plane_angle> position_delta;
  joint.getData(currentPosition);
  
  JointSensedVelocity currentVelocity;
  quantity<si::angular_velocity> velocity_current;
  joint.getData(currentVelocity);
  
  std::list<int32> targetVelocities;
  
  for(unsigned int i = 0; i< positions.size(); i++) {
    if(i == 0){
      position_delta = positions[i] - currentPosition.angle; 
      velocity_current = currentVelocity.angularVelocity;
    }else{
      position_delta = positions[i] - positions[i-1];
      velocity_current = velocities[i-1];
  }
    LOG(error) << velocity_current;
    
    this->calculateVelocities(position_delta, velocities[i], velocity_current,  accelerations[i], targetVelocities);
  }
  
  //DEBUG: just to stop the joint
  targetVelocities.push_back(0);
  
  
  unsigned int jointNumber = joint.getJointNumber();
  
  ethercatMaster->setTrajectoryVelocities(targetVelocities, jointNumber);
  
  // Bouml preserved body end 000E84F1
}

/// Stops just the trajectory controller but not the joint movement
void JointTrajectoryController::stopTrajectory() {
  // Bouml preserved body begin 000E8571
  // Bouml preserved body end 000E8571
}

/// Stops the joint movement by decreasing the velocity until zero
void JointTrajectoryController::stopJointMovement() {
  // Bouml preserved body begin 000E85F1
  // Bouml preserved body end 000E85F1
}

/// returns true if the trajectory controller is active
bool JointTrajectoryController::isTrajectoryControllerActive() {
  // Bouml preserved body begin 000E86F1
  return false;
  // Bouml preserved body end 000E86F1
}

void JointTrajectoryController::calculateVelocities(const quantity<plane_angle>& position_delta, const quantity<angular_velocity>& velocity, const quantity<angular_velocity>& velocity_current, const quantity<angular_acceleration>& acceleration, std::list<int32>& targetVelocities) {
  // Bouml preserved body begin 000EA071

  quantity<boost::units::si::time> totalTime;
  totalTime = (position_delta/velocity); //+ (velocity/(acceleration * 2.0)); //old
  
 // totalTime = (position_delta/velocity) + ((2.0*velocity*velocity_delta + (velocity_delta*velocity_delta))/(acceleration * 2.0 * velocity));
  
 // totalTime = (position_delta/velocity) + (velocity_delta/acceleration);
  
 // LOG(error) << "second part: " <<((2.0*velocity*velocity_delta + (velocity_delta*velocity_delta))/(acceleration * 2.0 * velocity));
  
  quantity<boost::units::si::time> accelerationTime;
  accelerationTime = (velocity - velocity_current)/acceleration;
  totalTime = totalTime + (accelerationTime/2.0);
  
  std::cout << "totalTime: " << totalTime << std::endl;
  std::cout << "accelerationTime: " << accelerationTime << std::endl;
  
  int totalTimeMili = (int)round(totalTime.value() * 1000);
  int accelerationTimeMili = (int)round(accelerationTime.value() * 1000);
  int32 vel;
  
  double targetVelocity; // radian_per_seconds
  
  for(int i= 1; i<=totalTimeMili;i++){
    if(i < accelerationTimeMili){
      targetVelocity = velocity_current.value() + (acceleration.value()) * ((double)i/1000.0);
     // LOG(error) << "targetVelocity ACC: " <<targetVelocity;
    }else{
      targetVelocity = velocity.value();
    }
    vel = (int32) round((targetVelocity / (gearRatio * 2.0 * M_PI)) * 60.0);
   // LOG(error) << "targetVelocity: " <<vel;
    targetVelocities.push_back(vel); //randiand per second in rpm
    ///TODO check inverse movement direction parameter
  }
  // Bouml preserved body end 000EA071
}


} // namespace youbot
