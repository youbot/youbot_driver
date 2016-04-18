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
#include "youbot_driver/youbot/GripperDataTrace.hpp"
namespace youbot {

GripperDataTrace::GripperDataTrace(YouBotGripperBar& youBotGripperBar, const std::string Name, const bool overwriteFiles):gripperBar(youBotGripperBar) {
  // Bouml preserved body begin 00101BF1

    roundsPerMinuteSetpoint.rpm = 0;
    PWMSetpoint.pwm = 0;
    encoderSetpoint.encoderTicks = 0;
    this->name = Name;
    if(Name != ""){
      this->path = Name;
      this->path.append("/");
    }
    char input = 0;
		
    if(boost::filesystem::exists((path+"gripperDataTrace").c_str())){
			while(input != 'y' && input != 'n' && overwriteFiles == false){
				std::cout << "Do you want to overwrite the existing files? [n/y]" << std::endl; 

				input = getchar();

				if(input == 'n'){
					throw std::runtime_error("Will not overwrite files!");
				}
			}

    }else{
      boost::filesystem::path rootPath (this->path);

      if ( !boost::filesystem::exists(this->path) ){
        if ( !boost::filesystem::create_directory( rootPath ))
          throw std::runtime_error("could not create folder: " + this->path);
      }
    }
    
  // Bouml preserved body end 00101BF1
}

GripperDataTrace::~GripperDataTrace() {
  // Bouml preserved body begin 00101C71
  // Bouml preserved body end 00101C71
}

void GripperDataTrace::startTrace(const std::string parameterName, const std::string unit) {
  // Bouml preserved body begin 00101CF1

    std::string parameterString;
    timeDurationMicroSec = 0;
    
   
    file.open((path+"gripperDataTrace").c_str(), std::fstream::out | std::fstream::trunc);
    
    ptime today;
    today = second_clock::local_time();
    
    file << "# Name: " << this->name << std::endl;
    
    file << "# Date: " << boost::posix_time::to_simple_string(today) << std::endl;
    
    file << "# time [milliseconds]"
            << " " << parameterName
            << std::endl;

    parametersBeginTraceFile.open((path+"ParametersAtBegin").c_str(), std::fstream::out | std::fstream::trunc);
    


 //   parameterVector.push_back(new GripperFirmwareVersion);
 //   parameterVector.push_back(new BarSpacingOffset);
 //   parameterVector.push_back(new MaxEncoderValue);
//    parameterVector.push_back(new MaxTravelDistance);
    parameterVector.push_back(new ActualPosition);
    parameterVector.push_back(new ActualVelocity);
    parameterVector.push_back(new ActualAcceleration);
    parameterVector.push_back(new ActualLoadValue);
    parameterVector.push_back(new ChopperBlankTime);
    parameterVector.push_back(new ChopperHysteresisDecrement);
    parameterVector.push_back(new ChopperHysteresisStart);
    parameterVector.push_back(new ChopperHysteresisEnd);
    parameterVector.push_back(new ChopperMode);
    parameterVector.push_back(new ChopperOffTime);
    parameterVector.push_back(new DoubleStepEnable);
    parameterVector.push_back(new ErrorFlags);
    parameterVector.push_back(new Freewheeling);
    parameterVector.push_back(new MaximumAcceleration);
    parameterVector.push_back(new MaximumCurrent);
    parameterVector.push_back(new MaximumPositioningSpeed);
    parameterVector.push_back(new MicrostepResolution);
    parameterVector.push_back(new PowerDownDelay);
    parameterVector.push_back(new PulseDivisor);
    parameterVector.push_back(new RampDivisor);
    parameterVector.push_back(new RampMode);
    parameterVector.push_back(new ShortDetectionTimer);
    parameterVector.push_back(new ShortProtectionDisable);
    parameterVector.push_back(new SlopeControlHighSide);
    parameterVector.push_back(new SlopeControlLowSide);
    parameterVector.push_back(new SmartEnergyActualCurrent);
    parameterVector.push_back(new SmartEnergyCurrentDownStep);
    parameterVector.push_back(new SmartEnergyCurrentMinimum);
    parameterVector.push_back(new SmartEnergyCurrentUpStep);
    parameterVector.push_back(new SmartEnergyHysteresis);
    parameterVector.push_back(new SmartEnergyHysteresisStart);
    parameterVector.push_back(new SmartEnergySlowRunCurrent);
    parameterVector.push_back(new SmartEnergyThresholdSpeed);
    parameterVector.push_back(new StallGuard2FilterEnable);
    parameterVector.push_back(new StallGuard2Threshold);
    parameterVector.push_back(new StandbyCurrent);
    parameterVector.push_back(new StepInterpolationEnable);
    parameterVector.push_back(new StopOnStall);
    parameterVector.push_back(new Vsense);
    parameterVector.push_back(new MinimumSpeed);

    parametersBeginTraceFile << "Name: " << this->name << std::endl;
    parametersBeginTraceFile << "Date: " << boost::posix_time::to_simple_string(today) << std::endl;

   
    for (unsigned int i = 0; i < parameterVector.size(); i++) {
      gripperBar.getConfigurationParameter(*(parameterVector[i]));
      parameterVector[i]->toString(parameterString);
      //   std::cout << parameterString << std::endl;
      parametersBeginTraceFile << parameterString << std::endl;
    }
    parametersBeginTraceFile.close();


    traceStartTime = microsec_clock::local_time();
  // Bouml preserved body end 00101CF1
}

void GripperDataTrace::stopTrace() {
  // Bouml preserved body begin 00101D71
    file.close();

    parametersEndTraceFile.open((path+"ParametersAfterTrace").c_str(), std::fstream::out | std::fstream::trunc);
    std::string parameterString;
    
    parametersEndTraceFile << "Name: " << this->name << std::endl;
    ptime today;
    today = second_clock::local_time();
    parametersEndTraceFile << "Date: " << boost::posix_time::to_simple_string(today) << std::endl;
    

    for (unsigned int i = 0; i < parameterVector.size(); i++) {
      gripperBar.getConfigurationParameter(*(parameterVector[i]));
      parameterVector[i]->toString(parameterString);
      parametersEndTraceFile << parameterString << std::endl;
      delete parameterVector[i];
    }
    
    
    parametersEndTraceFile.close();
  // Bouml preserved body end 00101D71
}

void GripperDataTrace::plotTrace() {
  // Bouml preserved body begin 00101DF1
  
    std::string executeString = "cd ";
    executeString.append(path);
    executeString.append("; gnuplot ../../GripperGnuPlotConfig");
    // > /dev/null 2>&1");
    std::system(executeString.c_str());
  // Bouml preserved body end 00101DF1
}

void GripperDataTrace::updateTrace(const double parameterValue) {
  // Bouml preserved body begin 001021F1
    timeDuration = microsec_clock::local_time() - traceStartTime;
    timeDurationMicroSec = timeDuration.total_milliseconds();
    file << timeDurationMicroSec << " " << parameterValue << std::endl;
  // Bouml preserved body end 001021F1
}

unsigned long GripperDataTrace::getTimeDurationMilliSec() {
  // Bouml preserved body begin 00102271
  return timeDurationMicroSec;
  // Bouml preserved body end 00102271
}


} // namespace youbot
