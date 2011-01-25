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

#include "generic/Errors.hpp"
namespace youbot {

Errors::Errors() {
  // Bouml preserved body begin 000211F1
  iter = this->occurredErrors.begin();
  // Bouml preserved body end 000211F1
}

Errors::~Errors() {
  // Bouml preserved body begin 00021271
  this->occurredErrors.clear();
  // Bouml preserved body end 00021271
}

void Errors::getNextError(std::string& name, std::string& description) {
  // Bouml preserved body begin 00022AFC
  if(iter == this->occurredErrors.end()){
    iter = this->occurredErrors.begin();
  }else{
    iter++;
  }
  name = iter->first;
  description = iter->second;
  // Bouml preserved body end 00022AFC
}

void Errors::getAllErrors(map<std::string, std::string>& allErrors) {
  // Bouml preserved body begin 00022B7C
  allErrors = this->occurredErrors;
  // Bouml preserved body end 00022B7C
}

unsigned int Errors::getAmountOfErrors() {
  // Bouml preserved body begin 00021171
  return this->occurredErrors.size();
  // Bouml preserved body end 00021171
}

void Errors::addError(std::string name, std::string description) {
  // Bouml preserved body begin 000212F1
  this->occurredErrors[name] = description;
  //  std::cout << "ERROR: " << name << " " << description << std::endl;


  LOG(error) << name << ": " << description;

  // Bouml preserved body end 000212F1
}

void Errors::deleteAllErrors() {
  // Bouml preserved body begin 00021371
  this->occurredErrors.clear();
  // Bouml preserved body end 00021371
}

void Errors::printErrorsToConsole() {
  // Bouml preserved body begin 0002FA71
  map<std::string,std::string>::iterator iterator;;
  for(iterator = this->occurredErrors.begin();iterator != this->occurredErrors.end(); iterator++){
    std::cout << iterator->first << ": " << iterator->second << std::endl;
  }

  // Bouml preserved body end 0002FA71
}


} // namespace youbot
