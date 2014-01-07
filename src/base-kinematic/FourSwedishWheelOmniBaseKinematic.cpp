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
#include "youbot_driver/base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
namespace youbot {

FourSwedishWheelOmniBaseKinematic::FourSwedishWheelOmniBaseKinematic() {
  // Bouml preserved body begin 000513F1
    this->lastWheelPositionInitialized = false;
  // Bouml preserved body end 000513F1
}

FourSwedishWheelOmniBaseKinematic::~FourSwedishWheelOmniBaseKinematic() {
  // Bouml preserved body begin 00051471
  // Bouml preserved body end 00051471
}

///Calculates from the cartesian velocity the individual wheel velocities 
///@param longitudinalVelocity is the forward or backward velocity
///@param transversalVelocity is the sideway velocity
///@param angularVelocity is the rotational velocity around the center of the YouBot
///@param wheelVelocities are the individual wheel velocities
void FourSwedishWheelOmniBaseKinematic::cartesianVelocityToWheelVelocities(const quantity<si::velocity>& longitudinalVelocity, const quantity<si::velocity>& transversalVelocity, const quantity<si::angular_velocity>& angularVelocity, std::vector<quantity<angular_velocity> >& wheelVelocities) {
  // Bouml preserved body begin 0004C071
    quantity<angular_velocity> RadPerSec_FromX;
    quantity<angular_velocity> RadPerSec_FromY;
    quantity<angular_velocity> RadPerSec_FromTheta;
    wheelVelocities.assign(4, RadPerSec_FromX);

    if (config.wheelRadius.value() == 0 || config.rotationRatio == 0 || config.slideRatio == 0) {
      throw std::out_of_range("The wheelRadius, RotationRatio or the SlideRatio are not allowed to be zero");
    }

    // RadPerSec_FromX = longitudinalVelocity / config.wheelRadius;
    RadPerSec_FromX = longitudinalVelocity.value() / config.wheelRadius.value() * radian_per_second;
    RadPerSec_FromY = transversalVelocity.value() / (config.wheelRadius.value() * config.slideRatio) * radian_per_second;

    // Calculate Rotation Component
    RadPerSec_FromTheta = ((config.lengthBetweenFrontAndRearWheels + config.lengthBetweenFrontWheels) / (2.0 * config.wheelRadius)) * angularVelocity;

    wheelVelocities[0] = -RadPerSec_FromX + RadPerSec_FromY + RadPerSec_FromTheta;
    wheelVelocities[1] = RadPerSec_FromX + RadPerSec_FromY + RadPerSec_FromTheta;
    wheelVelocities[2] = -RadPerSec_FromX - RadPerSec_FromY + RadPerSec_FromTheta;
    wheelVelocities[3] = RadPerSec_FromX - RadPerSec_FromY + RadPerSec_FromTheta;

    return;

  // Bouml preserved body end 0004C071
}

///Calculates from the wheel velocities the cartesian velocity
///@param wheelVelocities are the velocities of the individual wheels
///@param longitudinalVelocity is the forward or backward velocity
///@param transversalVelocity is the sideway velocity
///@param angularVelocity is the rotational velocity around the center of the YouBot
void FourSwedishWheelOmniBaseKinematic::wheelVelocitiesToCartesianVelocity(const std::vector<quantity<angular_velocity> >& wheelVelocities, quantity<si::velocity>& longitudinalVelocity, quantity<si::velocity>& transversalVelocity, quantity<angular_velocity>& angularVelocity) {
  // Bouml preserved body begin 0004C0F1

    if (wheelVelocities.size() < 4)
      throw std::out_of_range("To less wheel velocities");

    if (config.lengthBetweenFrontAndRearWheels.value() == 0 || config.lengthBetweenFrontWheels.value() == 0) {
      throw std::out_of_range("The lengthBetweenFrontAndRearWheels or the lengthBetweenFrontWheels are not allowed to be zero");
    }

    quantity<si::length> wheel_radius_per4 = config.wheelRadius / 4.0;

    quantity<si::length> geom_factor = (config.lengthBetweenFrontAndRearWheels / 2.0) + (config.lengthBetweenFrontWheels / 2.0);
    //now convert this to a vx,vy,vtheta
    longitudinalVelocity = (-wheelVelocities[0] + wheelVelocities[1] - wheelVelocities[2] + wheelVelocities[3]).value() * wheel_radius_per4.value() * meter_per_second;
    transversalVelocity = (wheelVelocities[0] + wheelVelocities[1] - wheelVelocities[2] - wheelVelocities[3]).value() * wheel_radius_per4.value() * meter_per_second;
    angularVelocity = (wheelVelocities[0] + wheelVelocities[1] + wheelVelocities[2] + wheelVelocities[3]) * (wheel_radius_per4 / geom_factor).value();

    return;
  // Bouml preserved body end 0004C0F1
}

///Calculates from the wheel positions the cartesian position
///@param wheelPositions are the individual positions of the wheels
///@param longitudinalPosition is the forward or backward position
///@param transversalPosition is the sideway position
///@param orientation is the rotation around the center
void FourSwedishWheelOmniBaseKinematic::wheelPositionsToCartesianPosition(const std::vector<quantity<plane_angle> >& wheelPositions, quantity<si::length>& longitudinalPosition, quantity<si::length>& transversalPosition, quantity<plane_angle>& orientation) {
  // Bouml preserved body begin 00051371

    if (wheelPositions.size() < 4)
      throw std::out_of_range("To less wheel positions");

     if (config.lengthBetweenFrontAndRearWheels.value() == 0 || config.lengthBetweenFrontWheels.value() == 0) {
      throw std::out_of_range("The lengthBetweenFrontAndRearWheels or the lengthBetweenFrontWheels are not allowed to be zero");
    }

    if (this->lastWheelPositionInitialized == false) {
      lastWheelPositions = wheelPositions;
      longitudinalPos = 0 * meter;
      transversalPos = 0 * meter;
      angle = 0 * radian;
      this->lastWheelPositionInitialized = true;
    }

    quantity<si::length> deltaLongitudinalPos;
    quantity<si::length> deltaTransversalPos;

    quantity<si::length> wheel_radius_per4 = config.wheelRadius / 4.0;

    quantity<si::length> geom_factor = (config.lengthBetweenFrontAndRearWheels / 2.0) + (config.lengthBetweenFrontWheels / 2.0);

    quantity<plane_angle> deltaPositionW1 = (wheelPositions[0] - lastWheelPositions[0]);
    quantity<plane_angle> deltaPositionW2 = (wheelPositions[1] - lastWheelPositions[1]);
    quantity<plane_angle> deltaPositionW3 = (wheelPositions[2] - lastWheelPositions[2]);
    quantity<plane_angle> deltaPositionW4 = (wheelPositions[3] - lastWheelPositions[3]);
    lastWheelPositions[0] = wheelPositions[0];
    lastWheelPositions[1] = wheelPositions[1];
    lastWheelPositions[2] = wheelPositions[2];
    lastWheelPositions[3] = wheelPositions[3];

    deltaLongitudinalPos = (-deltaPositionW1 + deltaPositionW2 - deltaPositionW3 + deltaPositionW4).value() * wheel_radius_per4.value() * meter;
    deltaTransversalPos = (deltaPositionW1 + deltaPositionW2 - deltaPositionW3 - deltaPositionW4).value() * wheel_radius_per4.value() * meter;
    angle += (deltaPositionW1 + deltaPositionW2 + deltaPositionW3 + deltaPositionW4) * (wheel_radius_per4 / geom_factor).value();

    longitudinalPos += deltaLongitudinalPos * cos(angle) - deltaTransversalPos * sin(angle);
    transversalPos += deltaLongitudinalPos * sin(angle) + deltaTransversalPos * cos(angle);

    longitudinalPosition = longitudinalPos;
    transversalPosition = transversalPos;
    orientation = angle;

    return;
  // Bouml preserved body end 00051371
}

///Calculates from the cartesian position the wheel positions
///@param longitudinalPosition is the forward or backward position
///@param transversalPosition is the sideway position
///@param orientation is the rotation around the center
///@param wheelPositions are the individual positions of the wheels
void FourSwedishWheelOmniBaseKinematic::cartesianPositionToWheelPositions(const quantity<si::length>& longitudinalPosition, const quantity<si::length>& transversalPosition, const quantity<plane_angle>& orientation, std::vector<quantity<plane_angle> >& wheelPositions) {
  // Bouml preserved body begin 000C08F1

    quantity<plane_angle> Rad_FromX;
    quantity<plane_angle> Rad_FromY;
    quantity<plane_angle> Rad_FromTheta;
    wheelPositions.assign(4, Rad_FromX);

    if (config.wheelRadius.value() == 0 || config.rotationRatio == 0 || config.slideRatio == 0) {
      throw std::out_of_range("The wheelRadius, RotationRatio or the SlideRatio are not allowed to be zero");
    }

    // RadPerSec_FromX = longitudinalVelocity / config.wheelRadius;
    Rad_FromX = longitudinalPosition.value() / config.wheelRadius.value() * radian;
    Rad_FromY = transversalPosition.value() / (config.wheelRadius.value() * config.slideRatio) * radian;

    // Calculate Rotation Component
    Rad_FromTheta = ((config.lengthBetweenFrontAndRearWheels + config.lengthBetweenFrontWheels) / (2.0 * config.wheelRadius)) * orientation;

    wheelPositions[0] = -Rad_FromX + Rad_FromY + Rad_FromTheta;
    wheelPositions[1] = Rad_FromX + Rad_FromY + Rad_FromTheta;
    wheelPositions[2] = -Rad_FromX - Rad_FromY + Rad_FromTheta;
    wheelPositions[3] = Rad_FromX - Rad_FromY + Rad_FromTheta;

    return;

  // Bouml preserved body end 000C08F1
}

void FourSwedishWheelOmniBaseKinematic::setConfiguration(const FourSwedishWheelOmniBaseKinematicConfiguration& configuration) {
  // Bouml preserved body begin 0004C171
    this->config = configuration;
  // Bouml preserved body end 0004C171
}

void FourSwedishWheelOmniBaseKinematic::getConfiguration(FourSwedishWheelOmniBaseKinematicConfiguration& configuration) const {
  // Bouml preserved body begin 0004C1F1
    configuration = this->config;
  // Bouml preserved body end 0004C1F1
}


} // namespace youbot
