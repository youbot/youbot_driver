#ifndef YOUBOT_ETHERCATMASTER_H
#define YOUBOT_ETHERCATMASTER_H

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
#include <string>
#include "youbot_driver/youbot/EthercatMasterInterface.hpp"
namespace youbot {

///////////////////////////////////////////////////////////////////////////////
/// The Ethercat Master factory
///////////////////////////////////////////////////////////////////////////////
class EthercatMaster {
friend class YouBotJoint;
friend class YouBotGripper;
friend class YouBotGripperBar;
  private:
    static EthercatMasterInterface* instance;

    EthercatMaster(){};

    EthercatMaster(const EthercatMaster& ) {};

    ~EthercatMaster(){};


  public:
    ///creates a instance of the singleton EthercatMaster if there is none and returns a reference to it
    ///@param configFile configuration file name incl. the extension
    ///@param configFilePath the path where the configuration is located with a / at the end
    ///@param ethercatMasterWithThread set it to false if you want to deactivate the communication thread
    static EthercatMasterInterface& getInstance(const std::string configFile = "youbot-ethercat.cfg", const std::string configFilePath = "../config/", const bool ethercatMasterWithThread = true);

    /// destroy the singleton instance by calling the destructor
    static void destroy();

};

} // namespace youbot
#endif
