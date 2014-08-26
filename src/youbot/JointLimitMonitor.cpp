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
#include "youbot_driver/youbot/JointLimitMonitor.hpp"
namespace youbot {

JointLimitMonitor::JointLimitMonitor(const YouBotJointStorage& jointParameters, const quantity<angular_acceleration>& jointAcceleration) {
  // Bouml preserved body begin 000FAB71
	this->storage = jointParameters;
	this->acceleration = jointAcceleration.value();  // rad/s^2
	brakingDistance = 0;
	isbraking = false;
	velocityWhenReachedLimit = 0;
					


  // Bouml preserved body end 000FAB71
}

JointLimitMonitor::~JointLimitMonitor() {
  // Bouml preserved body begin 000FABF1
  // Bouml preserved body end 000FABF1
}

JointLimitMonitor::JointLimitMonitor(const JointLimitMonitor & source) {
  // Bouml preserved body begin 000FAC71
	this->storage = source.storage;
  // Bouml preserved body end 000FAC71
}

JointLimitMonitor & JointLimitMonitor::operator=(const JointLimitMonitor & source) {
  // Bouml preserved body begin 000FACF1
	this->storage = source.storage;
	return *this;
  // Bouml preserved body end 000FACF1
}

void JointLimitMonitor::checkLimitsPositionControl(const quantity<plane_angle>& setpoint) {
  // Bouml preserved body begin 000FAD71
	if (storage.gearRatio == 0) {
      throw std::out_of_range("A Gear Ratio of zero is not allowed");
    }
    
    if (storage.encoderTicksPerRound == 0) {
      throw std::out_of_range("Zero Encoder Ticks per Round are not allowed");
    }
    if(storage.areLimitsActive){

      quantity<plane_angle> lowLimit = ((double) this->storage.lowerLimit / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
      quantity<plane_angle> upLimit = ((double) this->storage.upperLimit / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
      
      if (storage.inverseMovementDirection) {
        upLimit = ((double) -(this->storage.lowerLimit) / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
        lowLimit = ((double) -(this->storage.upperLimit) / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI) * radian;
      }
      

      if (!((setpoint < upLimit) && (setpoint > lowLimit))) {
        std::stringstream errorMessageStream;
        errorMessageStream << "The setpoint angle for joint "<< this->storage.jointName << " is out of range. The valid range is between " << lowLimit.value() << " and " << upLimit.value() << " and it  is: " << setpoint.value();
        throw std::out_of_range(errorMessageStream.str());
      }
    }
  // Bouml preserved body end 000FAD71
}

void JointLimitMonitor::checkLimitsEncoderPosition(const signed int& setpoint) {
  // Bouml preserved body begin 000FAF71
    if(storage.areLimitsActive){
      long upLimit = storage.upperLimit;
      long lowLimit = storage.lowerLimit;
      if (storage.inverseMovementDirection) {
        upLimit = -storage.lowerLimit;
        lowLimit = -storage.upperLimit;
      }

      if (!((setpoint < upLimit) && (setpoint > lowLimit))) {
        std::stringstream errorMessageStream;
        errorMessageStream << "The setpoint angle for joint "<< this->storage.jointName <<" is out of range. The valid range is between " << lowLimit << " and " << upLimit << " and it is: " << setpoint;
        throw std::out_of_range(errorMessageStream.str());
      }
    }
  // Bouml preserved body end 000FAF71
}

void JointLimitMonitor::checkLimitsProcessData(const SlaveMessageInput& messageInput, SlaveMessageOutput& messageOutput) {
  // Bouml preserved body begin 000FCAF1
    switch (messageOutput.controllerMode) {
      case POSITION_CONTROL:
        break;
      case VELOCITY_CONTROL:
        if (isbraking == false) {
          calculateBrakingDistance(messageInput);
        }

        if ((messageInput.actualPosition < bevorLowerLimit && !(messageOutput.value > 0)) || (messageInput.actualPosition > bevorUpperLimit && !(messageOutput.value < 0))) {
          //	messageOutput.value = velocityWhenReachedLimit * this->calculateDamping(messageInput.actualPosition);
          messageOutput.value = this->calculateBrakingVelocity(messageInput.actualPosition);
          isbraking = true;
        } else {
          isbraking = false;
        }

        break;
      case CURRENT_MODE:
        break; //disable limit checker for current mode
        /*
        if(isbraking == false){
          calculateBrakingDistance(messageInput);
        }
					
        if( (messageInput.actualPosition < bevorLowerLimit && !(messageOutput.value > 0)) || (messageInput.actualPosition > bevorUpperLimit && !(messageOutput.value < 0))) {
          messageOutput.value = this->calculateBrakingVelocity(messageInput.actualPosition);
          messageOutput.controllerMode = VELOCITY_CONTROL;
          isbraking = true;
        }else{
          isbraking = false;
        }

        break;
         */
      default:

        break;

    }
    
  // Bouml preserved body end 000FCAF1
}

double JointLimitMonitor::calculateDamping(const int actualPosition) {
  // Bouml preserved body begin 000FAFF1
	if(actualPosition <= storage.lowerLimit){
		return 0.0;
	}
	if(actualPosition >= storage.upperLimit){
		return 0.0;
	}
	if(actualPosition < bevorLowerLimit){
		return abs(((double)(actualPosition - storage.lowerLimit))/(bevorLowerLimit - storage.lowerLimit));
	}
	if(actualPosition > bevorUpperLimit){
		return abs(((double)(storage.upperLimit - actualPosition))/(storage.upperLimit - bevorUpperLimit));
	}
	return 0.0;
	
  // Bouml preserved body end 000FAFF1
}

void JointLimitMonitor::calculateBrakingDistance(const SlaveMessageInput& messageInput) {
  // Bouml preserved body begin 000FE471
		actualVelocityRPS = (((double) messageInput.actualVelocity / 60.0) * storage.gearRatio * 2.0 * M_PI); // radian_per_second;

		brakingDistance = (int) abs((((actualVelocityRPS * actualVelocityRPS) / (2.0 * acceleration)) * ((double) storage.encoderTicksPerRound / (2.0 * M_PI))) / storage.gearRatio);

		bevorLowerLimit = storage.lowerLimit + brakingDistance;
		bevorUpperLimit = storage.upperLimit - brakingDistance;
		velocityWhenReachedLimit = messageInput.actualVelocity;
  // Bouml preserved body end 000FE471
}

int JointLimitMonitor::calculateBrakingVelocity(const int actualPosition) {
  // Bouml preserved body begin 000FE4F1
	if(actualPosition <= storage.lowerLimit){
		return 0;
	}
	if(actualPosition >= storage.upperLimit){
		return 0;
	}
	if(actualPosition < bevorLowerLimit){
		distanceToLimit = ((double) (actualPosition - storage.lowerLimit) / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI);
		newVelocity =  -sqrt(2.0*acceleration* distanceToLimit);
		return (int) boost::math::round((newVelocity / (storage.gearRatio * 2.0 * M_PI)) * 60.0);
	}
	if(actualPosition > bevorUpperLimit){
		distanceToLimit = ((double) (storage.upperLimit - actualPosition) / storage.encoderTicksPerRound) * storage.gearRatio * (2.0 * M_PI);
		newVelocity =  sqrt(2.0*acceleration* distanceToLimit);
		return (int) boost::math::round((newVelocity / (storage.gearRatio * 2.0 * M_PI)) * 60.0);
	}
	return 0;
	
  // Bouml preserved body end 000FE4F1
}


} // namespace youbot
