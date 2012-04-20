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
    this->ISum = 0;
    this->DDiff = 0;
    this->PParameter = 10.0;
    this->IParameter = 1;
    this->DParameter = 0;
    this->last_pose_diff = 0;
  // Bouml preserved body end 000EA0F1
}

JointTrajectoryController::~JointTrajectoryController() {
  // Bouml preserved body begin 000EA171
  // Bouml preserved body end 000EA171
}

void JointTrajectoryController::setTrajectoryPositions(const std::list<int32>& targetPositions) {
  // Bouml preserved body begin 000EA1F1

    if (trajectoryPositionsBuffer1InUse == false) {
      {
        boost::mutex::scoped_lock dataMutex1(trajectoryPositionsBuffer1Mutex);
        this->trajectoryPositionsBuffer1 = targetPositions;

      }
    } else if (trajectoryPositionsBuffer2InUse == false) {
      {
        boost::mutex::scoped_lock dataMutex2(trajectoryPositionsBuffer2Mutex);
        this->trajectoryPositionsBuffer2 = targetPositions;
      }

    } else {
      LOG(error) << "Could not set the Trajectory!";
    }
  // Bouml preserved body end 000EA1F1
}

bool JointTrajectoryController::updateTrajectoryController(const SlaveMessageInput& actual, SlaveMessageOutput& velocity) {
  // Bouml preserved body begin 000EA271
  int32 targetPosition;
    {
      boost::mutex::scoped_lock dataMutex2(trajectoryPositionsBuffer2Mutex);
      {
      boost::mutex::scoped_lock dataMutex2(trajectoryPositionsBuffer1Mutex);

      if (!trajectoryPositionsBuffer1.empty() && !trajectoryPositionsBuffer1InUse && !trajectoryPositionsBuffer2InUse) {
        trajectoryPositionsBuffer1InUse = true;
      }

      if (!trajectoryPositionsBuffer1.empty() && trajectoryPositionsBuffer1InUse) {
        targetPosition = (trajectoryPositionsBuffer1).front();
        (trajectoryPositionsBuffer1).pop_front();
      } else {
        trajectoryPositionsBuffer1InUse = false;
      }
      
      if (!trajectoryPositionsBuffer2.empty() && !trajectoryPositionsBuffer1InUse && !trajectoryPositionsBuffer2InUse) {
        trajectoryPositionsBuffer2InUse = true;
      }

      if (!trajectoryPositionsBuffer2.empty() && trajectoryPositionsBuffer2InUse) {
        targetPosition = (trajectoryPositionsBuffer2).front();
        (trajectoryPositionsBuffer2).pop_front();
      } else {
        trajectoryPositionsBuffer2InUse = false;
      }
    }
    }
    
    if(!trajectoryPositionsBuffer1InUse && !trajectoryPositionsBuffer2InUse){
      return false;
    }
      
   
  ///////////////////////// Controller
  
    int32 pose_diff;
    pose_diff =  targetPosition - actual.actualPosition;
    int32 pose_diff_clipping = 65535;
    if(pose_diff > pose_diff_clipping)
      pose_diff = pose_diff_clipping;
    
    if(pose_diff < -pose_diff_clipping)
      pose_diff = -pose_diff_clipping;
    
    double ISum_clipping = 1000;
    this->ISum = this->ISum + pose_diff;
    
    
    if(ISum > ISum_clipping)
      ISum = ISum_clipping;
    
    if(ISum < -ISum_clipping)
      ISum = -ISum_clipping;
      
      
    this->DDiff = this->last_pose_diff - pose_diff;
    velocity.value = ((this->PParameter/256.0) * pose_diff) + ((this->IParameter/65536.0)* this->ISum) + ((this->DParameter/256.0) * this->DDiff);
    this->last_pose_diff =  pose_diff;
    
    velocity.controllerMode = VELOCITY_CONTROL;
  //  printf("pose_target: %10d  pose_diff: %10d   velout: %10d\n",targetPosition, pose_diff, velocity.value);
    
    return true;
  // Bouml preserved body end 000EA271
}

boost::mutex JointTrajectoryController::trajectoryPositionsBuffer1Mutex;

boost::mutex JointTrajectoryController::trajectoryPositionsBuffer2Mutex;


} // namespace youbot
