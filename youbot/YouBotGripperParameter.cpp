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
#include "youbot/YouBotGripperParameter.hpp"
namespace youbot {

YouBotGripperParameter::YouBotGripperParameter() {
  // Bouml preserved body begin 0005F0F1
  // Bouml preserved body end 0005F0F1
}

YouBotGripperParameter::~YouBotGripperParameter() {
  // Bouml preserved body begin 0005F171
  // Bouml preserved body end 0005F171
}

CalibrateGripper::CalibrateGripper() {
  // Bouml preserved body begin 0005F3F1
    this->name = "CalibrateGripper";
  // Bouml preserved body end 0005F3F1
}

CalibrateGripper::~CalibrateGripper() {
  // Bouml preserved body begin 0005F471
  // Bouml preserved body end 0005F471
}

void CalibrateGripper::getParameter(bool& parameter) const {
  // Bouml preserved body begin 0005F4F1
    parameter = this->value;
  // Bouml preserved body end 0005F4F1
}

void CalibrateGripper::setParameter(const bool parameter) {
  // Bouml preserved body begin 0005F571
    this->value = parameter;
  // Bouml preserved body end 0005F571
}

BarSpacingOffset::BarSpacingOffset() {
  // Bouml preserved body begin 0005FC71
    this->name = "BarSpacingOffset";
  // Bouml preserved body end 0005FC71
}

BarSpacingOffset::~BarSpacingOffset() {
  // Bouml preserved body begin 0005FCF1
  // Bouml preserved body end 0005FCF1
}

void BarSpacingOffset::getParameter(quantity<si::length>& parameter) const {
  // Bouml preserved body begin 0005FD71
    parameter = this->value;
  // Bouml preserved body end 0005FD71
}

void BarSpacingOffset::setParameter(const quantity<si::length>& parameter) {
  // Bouml preserved body begin 0005FDF1
  if(parameter > 1 *meter || parameter < 0 *meter){
    throw std::out_of_range("The Bar Spacing Offset is only allowed to be less than 1m and bigger than zero");
  }
    this->value = parameter;
  // Bouml preserved body end 0005FDF1
}

MaxEncoderValue::MaxEncoderValue() {
  // Bouml preserved body begin 00061B71
    this->name = "MaxEncoderValue";
  // Bouml preserved body end 00061B71
}

MaxEncoderValue::~MaxEncoderValue() {
  // Bouml preserved body begin 00061BF1
  // Bouml preserved body end 00061BF1
}

void MaxEncoderValue::getParameter(unsigned int& parameter) const {
  // Bouml preserved body begin 00061C71
    parameter = this->value;
  // Bouml preserved body end 00061C71
}

void MaxEncoderValue::setParameter(const unsigned int parameter) {
  // Bouml preserved body begin 00061CF1
    this->value = parameter;
  // Bouml preserved body end 00061CF1
}

MaxTravelDistance::MaxTravelDistance() {
  // Bouml preserved body begin 000618F1
    this->name = "MaxTravelDistance";
  // Bouml preserved body end 000618F1
}

MaxTravelDistance::~MaxTravelDistance() {
  // Bouml preserved body begin 00061971
  // Bouml preserved body end 00061971
}

void MaxTravelDistance::getParameter(quantity<si::length>& parameter) const {
  // Bouml preserved body begin 000619F1
    parameter = this->value;
  // Bouml preserved body end 000619F1
}

void MaxTravelDistance::setParameter(const quantity<si::length>& parameter) {
  // Bouml preserved body begin 00061A71
  if(parameter > 1 *meter || parameter < 0 *meter){
    throw std::out_of_range("The Max Travel Distance is only allowed to be less than 1m and bigger than zero");
  }
    this->value = parameter;
  // Bouml preserved body end 00061A71
}


} // namespace youbot
