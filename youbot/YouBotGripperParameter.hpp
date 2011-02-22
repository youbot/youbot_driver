#ifndef YOUBOT_YOUBOTGRIPPERPARAMETER_H
#define YOUBOT_YOUBOTGRIPPERPARAMETER_H

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
#include "generic/Logger.hpp"
#include "generic/Units.hpp"
#include "generic/Time.hpp"
#include "generic/Exceptions.hpp"
#include "generic-joint/JointParameter.hpp"
#include "generic-gripper/GripperParameter.hpp"
#include "youbot/ProtocolDefinitions.hpp"
#include "youbot/YouBotSlaveMsg.hpp"
#include "youbot/YouBotSlaveMailboxMsg.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// abstract youBot gripper parameter
///////////////////////////////////////////////////////////////////////////////
class YouBotGripperParameter : public GripperParameter {
friend class YouBotGripper;
  protected:
    YouBotGripperParameter();


  public:
    virtual ~YouBotGripperParameter();


  protected:
    virtual std::string getName() const = 0;

    std::string name;

};
///////////////////////////////////////////////////////////////////////////////
/// Calibrate the gripper
///////////////////////////////////////////////////////////////////////////////
class CalibrateGripper : public YouBotGripperParameter {
friend class YouBotGripper;
  public:
    CalibrateGripper();

    virtual ~CalibrateGripper();

    void getParameter(bool& parameter) const;

    void setParameter(const bool parameter);


  private:
    std::string getName() const {return this->name;};

    bool value;

    std::string name;

};
///////////////////////////////////////////////////////////////////////////////
/// Represents a bar spacing offset. It could be useful if the gripper can not be totally closed.
///////////////////////////////////////////////////////////////////////////////
class BarSpacingOffset : public YouBotGripperParameter {
friend class YouBotGripper;
  public:
    BarSpacingOffset();

    virtual ~BarSpacingOffset();

    void getParameter(quantity<si::length>& parameter) const;

    void setParameter(const quantity<si::length>& parameter);


  private:
    std::string getName() const {return this->name;};

    quantity<si::length> value;

    std::string name;

};
///////////////////////////////////////////////////////////////////////////////
/// The encoder value when the gripper has reached it's maximum bar spacing position
///////////////////////////////////////////////////////////////////////////////
class MaxEncoderValue : public YouBotGripperParameter {
friend class YouBotGripper;
  public:
    MaxEncoderValue();

    virtual ~MaxEncoderValue();

    void getParameter(unsigned int& parameter) const;

    void setParameter(const unsigned int parameter);


  private:
    std::string getName() const {return this->name;};

    unsigned int value;

    std::string name;

};
///////////////////////////////////////////////////////////////////////////////
/// The maximum bar spacing distance of the gripper
///////////////////////////////////////////////////////////////////////////////
class MaxTravelDistance : public YouBotGripperParameter {
friend class YouBotGripper;
  public:
    MaxTravelDistance();

    virtual ~MaxTravelDistance();

    void getParameter(quantity<si::length>& parameter) const;

    void setParameter(const quantity<si::length>& parameter);


  private:
    std::string getName() const {return this->name;};

    quantity<si::length> value;

    std::string name;

};

} // namespace youbot
#endif
