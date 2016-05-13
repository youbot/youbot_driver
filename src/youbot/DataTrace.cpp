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
#include "youbot_driver/youbot/DataTrace.hpp"
namespace youbot {

DataTrace::DataTrace(YouBotJoint& youBotJoint, const std::string Name, const bool overwriteFiles):joint(youBotJoint) {
  // Bouml preserved body begin 000C8F71

    roundsPerMinuteSetpoint.rpm = 0;
    PWMSetpoint.pwm = 0;
    encoderSetpoint.encoderTicks = 0;
    
    InverseMovementDirection invertDirectionParameter;
    joint.getConfigurationParameter(invertDirectionParameter);
    bool inverted = false;
    invertDirectionParameter.getParameter(inverted);
    if(inverted){
      invertDirection = -1;
    }else{
      invertDirection = 1;
    }
    
    this->name = Name;
    if(Name != ""){
      this->path = Name;
      this->path.append("/");
    }
    char input = 0;
		
    if(boost::filesystem::exists((path+"jointDataTrace").c_str())){
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
    
  // Bouml preserved body end 000C8F71
}

DataTrace::~DataTrace() {
  // Bouml preserved body begin 000C8FF1
  // Bouml preserved body end 000C8FF1
}

void DataTrace::startTrace() {
  // Bouml preserved body begin 000C93F1

    timeDurationMicroSec = 0;
    
   
    file.open((path+"jointDataTrace").c_str(), std::fstream::out | std::fstream::trunc);
    
    ptime today;
    today = second_clock::local_time();
    
    file << "# Name: " << this->name << std::endl;
    
    file << "# Date: " << boost::posix_time::to_simple_string(today) << std::endl;
    
    JointName jointName;
    FirmwareVersion firmwareParameter;
    std::string parameterString;
    joint.getConfigurationParameter(firmwareParameter);
    joint.getConfigurationParameter(jointName);
    jointName.toString(parameterString);
    file << "# " << parameterString << std::endl;
    firmwareParameter.toString(parameterString);
    file << "# " << parameterString << std::endl;
    
    
    file << "# time [milliseconds]"
            << " " << "angle setpoint [rad]"
            << " " << "velocity setpoint [rad/s]"
            << " " << "RPM setpoint"
            << " " << "current setpoint [A]"
            << " " << "torque setpoint [Nm]"
            << " " << "ramp generator setpoint [rad/s]"
            << " " << "encoder setpoint"

            << " " << "sensed angle [rad]"
            << " " << "sensed encoder ticks"
            << " " << "sensed velocity [rad/s]"
            << " " << "sensed RPM"
            << " " << "sensed current [A]"
            << " " << "sensed torque [Nm]"
            << " " << "actual PWM"

            << " " << "OVER_CURRENT" << " "
            << "UNDER_VOLTAGE" << " "
            << "OVER_VOLTAGE" << " "
            << "OVER_TEMPERATURE" << " "
            << "MOTOR_HALTED" << " "
            << "HALL_SENSOR_ERROR" << " "
            << "PWM_MODE_ACTIVE" << " "
            << "VELOCITY_MODE" << " "
            << "POSITION_MODE" << " "
            << "TORQUE_MODE" << " "
            << "POSITION_REACHED" << " "
            << "INITIALIZED" << " "
            << "TIMEOUT" << " "
            << "I2T_EXCEEDED" << " "
            << std::endl;

    parametersBeginTraceFile.open((path+"ParametersAtBegin").c_str(), std::fstream::out | std::fstream::trunc);
    


    parameterVector.push_back(new ActualMotorVoltage);
    //   parameterVector.push_back(new ErrorAndStatus);
    parameterVector.push_back(new ActualMotorDriverTemperature);
    parameterVector.push_back(new I2tSum);
    parameterVector.push_back(new PositionError);
    parameterVector.push_back(new PositionErrorSum);
    parameterVector.push_back(new RampGeneratorSpeed);
    parameterVector.push_back(new VelocityError);
    parameterVector.push_back(new VelocityErrorSum);
    //   parameterVector.push_back(new CalibrateJoint);
    parameterVector.push_back(new DParameterFirstParametersPositionControl);
    parameterVector.push_back(new DParameterFirstParametersSpeedControl);
    parameterVector.push_back(new DParameterCurrentControl);
    parameterVector.push_back(new DParameterSecondParametersPositionControl);
    parameterVector.push_back(new DParameterSecondParametersSpeedControl);

    parameterVector.push_back(new IClippingParameterFirstParametersPositionControl);
    parameterVector.push_back(new IClippingParameterFirstParametersSpeedControl);
    parameterVector.push_back(new IClippingParameterCurrentControl);
    parameterVector.push_back(new IClippingParameterSecondParametersPositionControl);
    parameterVector.push_back(new IClippingParameterSecondParametersSpeedControl);
    //    parameterVector.push_back(new InitializeJoint);
    
    parameterVector.push_back(new IParameterFirstParametersPositionControl);
    parameterVector.push_back(new IParameterFirstParametersSpeedControl);
    parameterVector.push_back(new IParameterCurrentControl);
    parameterVector.push_back(new IParameterSecondParametersPositionControl);
    parameterVector.push_back(new IParameterSecondParametersSpeedControl);

    parameterVector.push_back(new MaximumPositioningVelocity);
    parameterVector.push_back(new MotorAcceleration);
    parameterVector.push_back(new PositionControlSwitchingThreshold);
    parameterVector.push_back(new PParameterFirstParametersPositionControl);
    parameterVector.push_back(new PParameterFirstParametersSpeedControl);
    parameterVector.push_back(new PParameterCurrentControl);
    parameterVector.push_back(new PParameterSecondParametersPositionControl);
    parameterVector.push_back(new PParameterSecondParametersSpeedControl);
    parameterVector.push_back(new RampGeneratorSpeedAndPositionControl);
    parameterVector.push_back(new SpeedControlSwitchingThreshold);

    parameterVector.push_back(new ActivateOvervoltageProtection);
    parameterVector.push_back(new ActualCommutationOffset);
    //   parameterVector.push_back(new ApproveProtectedParameters);
    parameterVector.push_back(new BEMFConstant);
    //   parameterVector.push_back(new ClearI2tExceededFlag);
    //   parameterVector.push_back(new ClearMotorControllerTimeoutFlag);
    parameterVector.push_back(new CommutationMode);
    parameterVector.push_back(new CommutationMotorCurrent);
    parameterVector.push_back(new CurrentControlLoopDelay);
    parameterVector.push_back(new EncoderResolution);
    parameterVector.push_back(new EncoderStopSwitch);
    parameterVector.push_back(new HallSensorPolarityReversal);
    parameterVector.push_back(new I2tExceedCounter);
    parameterVector.push_back(new I2tLimit);
    parameterVector.push_back(new InitializationMode);
    parameterVector.push_back(new InitSineDelay);
    parameterVector.push_back(new MassInertiaConstant);
    parameterVector.push_back(new MaximumMotorCurrent);
    parameterVector.push_back(new MaximumVelocityToSetPosition);
    parameterVector.push_back(new MotorCoilResistance);
    parameterVector.push_back(new MotorControllerTimeout);
    parameterVector.push_back(new MotorPoles);
    parameterVector.push_back(new OperationalTime);
    parameterVector.push_back(new PIDControlTime);
    parameterVector.push_back(new PositionTargetReachedDistance);
    parameterVector.push_back(new ReversingEncoderDirection);
    parameterVector.push_back(new SetEncoderCounterZeroAtNextNChannel);
    parameterVector.push_back(new SetEncoderCounterZeroAtNextSwitch);
    parameterVector.push_back(new SetEncoderCounterZeroOnlyOnce);
    parameterVector.push_back(new SineInitializationVelocity);
    parameterVector.push_back(new StopSwitchPolarity);
    parameterVector.push_back(new ThermalWindingTimeConstant);
    parameterVector.push_back(new VelocityThresholdForHallFX);
    parameterVector.push_back(new MotorHaltedVelocity);

//    apiParameterVector.push_back(new JointName);
//    apiParameterVector.push_back(new TorqueConstant);
//    apiParameterVector.push_back(new JointLimits);
//    apiParameterVector.push_back(new GearRatio);
//    apiParameterVector.push_back(new EncoderTicksPerRound);
//    apiParameterVector.push_back(new InverseMovementDirection);

//    for (int i = 0; i < apiParameterVector.size(); i++) {
//      joint.getConfigurationParameter(*(apiParameterVector[i]));
//      apiParameterVector[i]->toString(parameterString);
//      //   std::cout << parameterString << std::endl;
//      parametersBeginTraceFile << parameterString << std::endl;
//    }


    parametersBeginTraceFile << "Name: " << this->name << std::endl;
    parametersBeginTraceFile << "Date: " << boost::posix_time::to_simple_string(today) << std::endl;
  //  joint.getConfigurationParameter(jointName);
    jointName.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersBeginTraceFile << parameterString << std::endl;
    firmwareParameter.toString(parameterString);
    parametersBeginTraceFile << parameterString << std::endl;

    TorqueConstant torqueconst;
    joint.getConfigurationParameter(torqueconst);
    torqueconst.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersBeginTraceFile << parameterString << std::endl;

    JointLimits jointLimits;
    joint.getConfigurationParameter(jointLimits);
    jointLimits.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersBeginTraceFile << parameterString << std::endl;

    EncoderTicksPerRound encoderTicksPerRound;
    joint.getConfigurationParameter(encoderTicksPerRound);
    encoderTicksPerRound.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersBeginTraceFile << parameterString << std::endl;

    GearRatio gearRatio;
    joint.getConfigurationParameter(gearRatio);
    gearRatio.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersBeginTraceFile << parameterString << std::endl;
    
    InverseMovementDirection inverseMovementDirection;
    joint.getConfigurationParameter(inverseMovementDirection);
    inverseMovementDirection.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersBeginTraceFile << parameterString << std::endl;
    
    
    for (unsigned int i = 0; i < parameterVector.size(); i++) {
      joint.getConfigurationParameter(*(parameterVector[i]));
      parameterVector[i]->toString(parameterString);
      //   std::cout << parameterString << std::endl;
      parametersBeginTraceFile << parameterString << std::endl;
    }
    parametersBeginTraceFile.close();


    traceStartTime = microsec_clock::local_time();
  // Bouml preserved body end 000C93F1
}

void DataTrace::stopTrace() {
  // Bouml preserved body begin 000C9471
    file.close();

    parametersEndTraceFile.open((path+"ParametersAfterTrace").c_str(), std::fstream::out | std::fstream::trunc);
    std::string parameterString;
    
    parametersEndTraceFile << "Name: " << this->name << std::endl;
    ptime today;
    today = second_clock::local_time();
    parametersEndTraceFile << "Date: " << boost::posix_time::to_simple_string(today) << std::endl;
    
        JointName jointName;
    joint.getConfigurationParameter(jointName);
    jointName.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersEndTraceFile << parameterString << std::endl;
    
    FirmwareVersion firmwareParameter;
    joint.getConfigurationParameter(firmwareParameter);
    firmwareParameter.toString(parameterString);
    parametersEndTraceFile << parameterString << std::endl;

    TorqueConstant torqueconst;
    joint.getConfigurationParameter(torqueconst);
    torqueconst.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersEndTraceFile << parameterString << std::endl;

    JointLimits jointLimits;
    joint.getConfigurationParameter(jointLimits);
    jointLimits.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersEndTraceFile << parameterString << std::endl;

    EncoderTicksPerRound encoderTicksPerRound;
    joint.getConfigurationParameter(encoderTicksPerRound);
    encoderTicksPerRound.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersEndTraceFile << parameterString << std::endl;

    GearRatio gearRatio;
    joint.getConfigurationParameter(gearRatio);
    gearRatio.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersEndTraceFile << parameterString << std::endl;
    
    InverseMovementDirection inverseMovementDirection;
    joint.getConfigurationParameter(inverseMovementDirection);
    inverseMovementDirection.toString(parameterString);
    //   std::cout << parameterString << std::endl;
    parametersEndTraceFile << parameterString << std::endl;
    
    for (unsigned int i = 0; i < parameterVector.size(); i++) {
      joint.getConfigurationParameter(*(parameterVector[i]));
      parameterVector[i]->toString(parameterString);
      parametersEndTraceFile << parameterString << std::endl;
      delete parameterVector[i];
    }
    
    
    parametersEndTraceFile.close();
  // Bouml preserved body end 000C9471
}

void DataTrace::plotTrace() {
  // Bouml preserved body begin 000C9571
  
    std::string executeString = "cd ";
    executeString.append(path);
    executeString.append("; gnuplot ../../gnuplotconfig > /dev/null 2>&1");
    std::system(executeString.c_str());
  // Bouml preserved body end 000C9571
}

void DataTrace::updateTrace(const JointAngleSetpoint& setpoint) {
  // Bouml preserved body begin 000C9071
    angleSetpoint = setpoint;
    controllerMode = POSITION_CONTROL_RAD;
    this->update();
  // Bouml preserved body end 000C9071
}

void DataTrace::updateTrace(const JointVelocitySetpoint& setpoint) {
  // Bouml preserved body begin 000C90F1
    velocitySetpoint = setpoint;
    controllerMode = VELOCITY_CONTROL_RAD_SEC;
    this->update();
  // Bouml preserved body end 000C90F1
}

void DataTrace::updateTrace(const JointRoundsPerMinuteSetpoint& setpoint) {
  // Bouml preserved body begin 000C9171
    roundsPerMinuteSetpoint = setpoint;
    controllerMode = VELOCITY_CONTROL_RPM;
    this->update();
  // Bouml preserved body end 000C9171
}

void DataTrace::updateTrace(const JointCurrentSetpoint& setpoint) {
  // Bouml preserved body begin 000C91F1
    currentSetpoint = setpoint;
    controllerMode = CURRENT_CONTROL_MODE;
    this->update();
  // Bouml preserved body end 000C91F1
}

void DataTrace::updateTrace(const JointTorqueSetpoint& setpoint) {
  // Bouml preserved body begin 000C9271
    torqueSetpoint = setpoint;
    controllerMode = TORQUE_CONTROL_MODE;
    this->update();
  // Bouml preserved body end 000C9271
}

void DataTrace::updateTrace(const JointEncoderSetpoint& setpoint) {
  // Bouml preserved body begin 000C9371
    encoderSetpoint = setpoint;
    controllerMode = POSITION_CONTROL_ENC;
    this->update();
  // Bouml preserved body end 000C9371
}

void DataTrace::updateTrace() {
  // Bouml preserved body begin 000EEC71
    YouBotSlaveMsg message;
    joint.getData(message);
    switch (message.stctOutput.controllerMode) {
      case MOTOR_STOP:
        controllerMode = NOT_DEFINED;
        break;
      case POSITION_CONTROL:
        encoderSetpoint.encoderTicks = message.stctOutput.value * invertDirection;
        controllerMode = POSITION_CONTROL_ENC;
        break;
      case VELOCITY_CONTROL:
        roundsPerMinuteSetpoint.rpm = message.stctOutput.value * invertDirection;
        controllerMode = VELOCITY_CONTROL_RPM;
        break;
      case NO_MORE_ACTION:
        controllerMode = NOT_DEFINED;
        break;
      case SET_POSITION_TO_REFERENCE:
        controllerMode = NOT_DEFINED;
        break;
      case CURRENT_MODE:
        currentSetpoint.current = (double)message.stctOutput.value /1000.0  * invertDirection* ampere;
        controllerMode = CURRENT_CONTROL_MODE;
        break;
      default:
        controllerMode = NOT_DEFINED;
        break;
    };


    this->update();
  // Bouml preserved body end 000EEC71
}

unsigned long DataTrace::getTimeDurationMilliSec() {
  // Bouml preserved body begin 000DCFF1
  return timeDurationMicroSec;
  // Bouml preserved body end 000DCFF1
}

void DataTrace::update() {
  // Bouml preserved body begin 000C94F1
    timeDuration = microsec_clock::local_time() - traceStartTime;
    timeDurationMicroSec = timeDuration.total_milliseconds();
    unsigned int statusFlags;
    joint.getStatus(statusFlags);
    joint.getData(sensedAngle);
    joint.getData(sensedEncoderTicks);
    joint.getData(sensedVelocity);
    joint.getData(sensedRoundsPerMinute);
    joint.getData(sensedCurrent);
    joint.getData(sensedTorque);
		joint.getData(targetAngle);
		joint.getData(targetVelocity);
		joint.getData(targetCurrent);
		joint.getData(rampGenSetpoint);
		
    std::stringstream angleSet, angleEncSet, velSet, velRPMSet, currentSet, pwmSet, torqueSet;

    switch (controllerMode) {
      case POSITION_CONTROL_RAD:
        angleSet << angleSetpoint.angle.value();
        angleEncSet << "NaN";
        velSet << targetVelocity.angularVelocity.value();
        velRPMSet << "NaN";
        currentSet << targetCurrent.current.value();
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case POSITION_CONTROL_ENC:
        angleSet << "NaN";
        angleEncSet << encoderSetpoint.encoderTicks;
        velSet << targetVelocity.angularVelocity.value();
        velRPMSet << "NaN";
        currentSet << targetCurrent.current.value();
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case VELOCITY_CONTROL_RAD_SEC:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << velocitySetpoint.angularVelocity.value();
        velRPMSet << "NaN";
        currentSet << targetCurrent.current.value();
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case VELOCITY_CONTROL_RPM:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << roundsPerMinuteSetpoint.rpm;
        currentSet << targetCurrent.current.value();
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case CURRENT_CONTROL_MODE:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << currentSetpoint.current.value();
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case TORQUE_CONTROL_MODE:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << torqueSetpoint.torque.value();
        break;
      case NOT_DEFINED:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
    };



    file << timeDurationMicroSec //1
            << " " << angleSet.str() //2
            << " " << velSet.str() //3
            << " " << velRPMSet.str() //4
            << " " << currentSet.str() //5
            << " " << torqueSet.str() //6
            << " " << rampGenSetpoint.angularVelocity.value() //7
            << " " << angleEncSet.str() //8

            << " " << sensedAngle.angle.value() //9
            << " " << sensedEncoderTicks.encoderTicks //10
            << " " << sensedVelocity.angularVelocity.value() //11
            << " " << sensedRoundsPerMinute.rpm //12
            << " " << sensedCurrent.current.value() //13
            << " " << sensedTorque.torque.value() //14
            << " " << "0" //15  //dummy has been pwm

            << " " << bool(statusFlags & OVER_CURRENT) << " " //16
            << bool(statusFlags & UNDER_VOLTAGE) << " " //17
            << bool(statusFlags & OVER_VOLTAGE) << " " //18
            << bool(statusFlags & OVER_TEMPERATURE) << " " //19
            << bool(statusFlags & MOTOR_HALTED) << " " //20
            << bool(statusFlags & HALL_SENSOR_ERROR) << " " //21
            << "0" << " " //22 //dummy has been pwm
            << bool(statusFlags & VELOCITY_MODE) << " " //23
            << bool(statusFlags & POSITION_MODE) << " " //24
            << bool(statusFlags & TORQUE_MODE) << " " //25
            << bool(statusFlags & POSITION_REACHED) << " " //26
            << bool(statusFlags & INITIALIZED) << " " //27
            << bool(statusFlags & TIMEOUT) << " " //28
            << bool(statusFlags & I2T_EXCEEDED) //29
            << std::endl;


  // Bouml preserved body end 000C94F1
}


} // namespace youbot
