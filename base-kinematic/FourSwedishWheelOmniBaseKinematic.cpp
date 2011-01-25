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
#include "base-kinematic/FourSwedishWheelOmniBaseKinematic.hpp"
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

void FourSwedishWheelOmniBaseKinematic::cartesianVelocityToWheelVelocities(const quantity<si::velocity>& longitudinalVelocity, const quantity<si::velocity>& transversalVelocity, const quantity<si::angular_velocity>& angularVelocity, std::vector<quantity<angular_velocity> >& wheelVelocities) {
  // Bouml preserved body begin 0004C071
    quantity<angular_velocity> RadPerSec_FromX;
    quantity<angular_velocity> RadPerSec_FromY;
    quantity<angular_velocity> RadPerSec_FromTheta;
    quantity<si::length> Perimeter;
    quantity<si::length> OneWheelRotation;
    wheelVelocities.assign(4, RadPerSec_FromX);

    if (config.wheelRadius.value() == 0 || config.rotationRatio == 0 || config.slideRatio == 0) {
      throw std::out_of_range("The wheelRadius, RotationRatio or the SlideRatio are not allowed to be zero");
    }

    // RadPerSec_FromX = longitudinalVelocity / config.wheelRadius;
    RadPerSec_FromX = longitudinalVelocity.value() / config.wheelRadius.value() * radian_per_second;
    RadPerSec_FromY = transversalVelocity.value() / (config.wheelRadius.value() * config.slideRatio) * radian_per_second;

    // Calculate Rotation Component
    Perimeter = (root < 2 > ((config.lengthBetweenFrontAndRearWheels * config.lengthBetweenFrontAndRearWheels) + (config.lengthBetweenFrontWheels * config.lengthBetweenFrontWheels))) * M_PI;
    OneWheelRotation = config.rotationRatio * config.wheelRadius * 2.0 * M_PI;
    RadPerSec_FromTheta = angularVelocity * (Perimeter / OneWheelRotation);


    wheelVelocities[0] = -RadPerSec_FromX + RadPerSec_FromY + RadPerSec_FromTheta;
    wheelVelocities[1] = RadPerSec_FromX + RadPerSec_FromY + RadPerSec_FromTheta;
    wheelVelocities[2] = -RadPerSec_FromX - RadPerSec_FromY + RadPerSec_FromTheta;
    wheelVelocities[3] = RadPerSec_FromX - RadPerSec_FromY + RadPerSec_FromTheta;

    return;

  // Bouml preserved body end 0004C071
}

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

  // Bouml preserved body end 0004C0F1
}

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

    longitudinalPos += (-deltaPositionW1 + deltaPositionW2 - deltaPositionW3 + deltaPositionW4).value() * wheel_radius_per4.value() * meter;
    transversalPos += (deltaPositionW1 + deltaPositionW2 - deltaPositionW3 - deltaPositionW4).value() * wheel_radius_per4.value() * meter;
    angle += (deltaPositionW1 + deltaPositionW2 + deltaPositionW3 + deltaPositionW4) * (wheel_radius_per4 / geom_factor).value();


    longitudinalPosition = longitudinalPos;
    transversalPosition = transversalPos;
    orientation = angle;
  // Bouml preserved body end 00051371
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
