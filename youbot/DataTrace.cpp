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
#include "youbot/DataTrace.hpp"
namespace youbot {

DataTrace::DataTrace(YouBotJoint& youBotJoint):joint(youBotJoint) {
  // Bouml preserved body begin 000C8F71

    roundsPerMinuteSetpoint.rpm = 0;
    PWMSetpoint.pwm = 0;
    encoderSetpoint.encoderTicks = 0;
  // Bouml preserved body end 000C8F71
}

DataTrace::~DataTrace() {
  // Bouml preserved body begin 000C8FF1
  // Bouml preserved body end 000C8FF1
}

void DataTrace::startTrace() {
  // Bouml preserved body begin 000C93F1


    file.open("jointDataTrace", std::fstream::out | std::fstream::trunc);

    file << "# time [milliseconds]"
            << " " << "angle setpoint [rad]"
            << " " << "velocity setpoint [rad/s]"
            << " " << "RPM setpoint"
            << " " << "current setpoint [A]"
            << " " << "torque setpoint [Nm]"
            << " " << "PWM setpoint"
            << " " << "encoder setpoint"

            << " " << "sensed angle [rad]"
            << " " << "sensed encoder ticks"
            << " " << "sensed velocity [rad/s]"
            << " " << "sensed RPM"
            << " " << "sensed current [A]"
            << " " << "sensed torque [Nm]"

            << " " << "OVER_CURRENT" << " "
            << "UNDER_VOLTAGE" << " "
            << "OVER_VOLTAGE" << " "
            << "OVER_TEMPERATURE" << " "
            << "MOTOR_HALTED" << " "
            << "HALL_SENSOR_ERROR" << " "
            << "ENCODER_ERROR" << " "
            << "INITIALIZATION_ERROR" << " "
            << "PWM_MODE_ACTIVE" << " "
            << "VELOCITY_MODE" << " "
            << "POSITION_MODE" << " "
            << "TORQUE_MODE" << " "
            << "EMERGENCY_STOP" << " "
            << "FREERUNNING" << " "
            << "POSITION_REACHED" << " "
            << "INITIALIZED" << " "
            << "TIMEOUT" << " "
            << "I2T_EXCEEDED" << std::endl;

    parametersBeginTraceFile.open("ParametersAtBegin", std::fstream::out | std::fstream::trunc);
    std::string parameterString;
    parameterVector.push_back(new ActualMotorVoltage);
    parameterVector.push_back(new ActualPWMDutyCycle);
    //   parameterVector.push_back(new ErrorAndStatus);
    parameterVector.push_back(new I2tSum);
    parameterVector.push_back(new PositionError);
    parameterVector.push_back(new PositionErrorSum);
    parameterVector.push_back(new RampGeneratorSpeed);
    parameterVector.push_back(new VelocityError);
    parameterVector.push_back(new VelocityErrorSum);
    //   parameterVector.push_back(new CalibrateJoint);
    parameterVector.push_back(new CurrentControlSwitchingThreshold);
    parameterVector.push_back(new DParameterFirstParametersCurrentControl);
    parameterVector.push_back(new DParameterFirstParametersPositionControl);
    parameterVector.push_back(new DParameterFirstParametersSpeedControl);
    parameterVector.push_back(new DParameterSecondParametersCurrentControl);
    parameterVector.push_back(new DParameterSecondParametersPositionControl);
    parameterVector.push_back(new DParameterSecondParametersSpeedControl);
    parameterVector.push_back(new EncoderTicksPerRound);
    //   parameterVector.push_back(new FirmwareVersion);
    parameterVector.push_back(new GearRatio);
    parameterVector.push_back(new IClippingParameterFirstParametersCurrentControl);
    parameterVector.push_back(new IClippingParameterFirstParametersPositionControl);
    parameterVector.push_back(new IClippingParameterFirstParametersSpeedControl);
    parameterVector.push_back(new IClippingParameterSecondParametersCurrentControl);
    parameterVector.push_back(new IClippingParameterSecondParametersPositionControl);
    parameterVector.push_back(new IClippingParameterSecondParametersSpeedControl);
    //    parameterVector.push_back(new InitializeJoint);
    parameterVector.push_back(new InverseMovementDirection);
    parameterVector.push_back(new IParameterFirstParametersCurrentControl);
    parameterVector.push_back(new IParameterFirstParametersPositionControl);
    parameterVector.push_back(new IParameterFirstParametersSpeedControl);
    parameterVector.push_back(new IParameterSecondParametersCurrentControl);
    parameterVector.push_back(new IParameterSecondParametersPositionControl);
    parameterVector.push_back(new IParameterSecondParametersSpeedControl);
    parameterVector.push_back(new JointLimits);
    parameterVector.push_back(new JointName);
    parameterVector.push_back(new MaximumPositioningVelocity);
    parameterVector.push_back(new MotorAcceleration);
    parameterVector.push_back(new PositionControlSwitchingThreshold);
    parameterVector.push_back(new PParameterFirstParametersCurrentControl);
    parameterVector.push_back(new PParameterFirstParametersPositionControl);
    parameterVector.push_back(new PParameterFirstParametersSpeedControl);
    parameterVector.push_back(new PParameterSecondParametersCurrentControl);
    parameterVector.push_back(new PParameterSecondParametersPositionControl);
    parameterVector.push_back(new PParameterSecondParametersSpeedControl);
    parameterVector.push_back(new RampGeneratorSpeedAndPositionControl);
    parameterVector.push_back(new SpeedControlSwitchingThreshold);
    parameterVector.push_back(new TorqueConstant);
    parameterVector.push_back(new ActivateOvervoltageProtection);
    parameterVector.push_back(new ActualCommutationOffset);
    //   parameterVector.push_back(new ApproveProtectedParameters);
    parameterVector.push_back(new BEMFConstant);
    //   parameterVector.push_back(new ClearI2tExceededFlag);
    parameterVector.push_back(new ClearISumIfPWMReachesMaximum);
    //   parameterVector.push_back(new ClearMotorControllerTimeoutFlag);
    parameterVector.push_back(new ClearTargetDistance);
    parameterVector.push_back(new CommutationCompensationClockwise);
    parameterVector.push_back(new CommutationCompensationCounterClockwise);
    parameterVector.push_back(new CommutationMode);
    parameterVector.push_back(new CommutationMotorCurrent);
    parameterVector.push_back(new CurrentControlLoopDelay);
    parameterVector.push_back(new EncoderNullPolarity);
    parameterVector.push_back(new EncoderResolution);
    parameterVector.push_back(new EncoderStopSwitch);
    parameterVector.push_back(new HallSensorPolarityReversal);
    parameterVector.push_back(new I2tExceedCounter);
    parameterVector.push_back(new I2tLimit);
    parameterVector.push_back(new InitializationMode);
    parameterVector.push_back(new InitSineDelay);
    parameterVector.push_back(new MassInertiaConstant);
    parameterVector.push_back(new MaximumMotorCurrent);
    parameterVector.push_back(new MaximumPWMChangePerPIDInterval);
    parameterVector.push_back(new MaximumVelocityToSetPosition);
    parameterVector.push_back(new MotorCoilResistance);
    parameterVector.push_back(new MotorContollerGearRatio);
    parameterVector.push_back(new MotorControllerTimeout);
    parameterVector.push_back(new MotorPoles);
    parameterVector.push_back(new OperationalTime);
    parameterVector.push_back(new PIDControllerState);
    parameterVector.push_back(new PIDControlTime);
    parameterVector.push_back(new PositionTargetReachedDistance);
    parameterVector.push_back(new PWMHysteresis);
    parameterVector.push_back(new PWMLimit);
    parameterVector.push_back(new PWMSchemeBlockCommutation);
    //   parameterVector.push_back(new ReinitializationSinusoidalCommutation);
    parameterVector.push_back(new ReversingEncoderDirection);
    parameterVector.push_back(new SetEncoderCounterZeroAtNextNChannel);
    parameterVector.push_back(new SetEncoderCounterZeroAtNextSwitch);
    parameterVector.push_back(new SetEncoderCounterZeroOnlyOnce);
    parameterVector.push_back(new SineCompensationFactor);
    parameterVector.push_back(new SineInitializationVelocity);
    parameterVector.push_back(new StopSwitchPolarity);
    parameterVector.push_back(new ThermalWindingTimeConstant);

    for (int i = 0; i < parameterVector.size(); i++) {
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

    parametersEndTraceFile.open("ParametersAfterTrace", std::fstream::out | std::fstream::trunc);
    std::string parameterString;
    for (int i = 0; i < parameterVector.size(); i++) {
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
    std::system("gnuplot ../gnuplotconfig");
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
    controllerMode = CURRENT_MODE;
    this->update();
  // Bouml preserved body end 000C91F1
}

void DataTrace::updateTrace(const JointTorqueSetpoint& setpoint) {
  // Bouml preserved body begin 000C9271
    torqueSetpoint = setpoint;
    controllerMode = TORQUE_MODE;
    this->update();
  // Bouml preserved body end 000C9271
}

void DataTrace::updateTrace(const JointPWMSetpoint& setpoint) {
  // Bouml preserved body begin 000C92F1
    PWMSetpoint = setpoint;
    controllerMode = PWM_MODE;
    this->update();
  // Bouml preserved body end 000C92F1
}

void DataTrace::updateTrace(const JointEncoderSetpoint& setpoint) {
  // Bouml preserved body begin 000C9371
    encoderSetpoint = setpoint;
    controllerMode = POSITION_CONTROL_ENC;
    this->update();
  // Bouml preserved body end 000C9371
}

void DataTrace::update() {
  // Bouml preserved body begin 000C94F1
    timeDuration = microsec_clock::local_time() - traceStartTime;
    timeDurationMicroSec = timeDuration.total_milliseconds();
    unsigned short statusFlags;

    joint.getStatus(statusFlags);
    joint.getData(sensedAngle);
    joint.getData(sensedEncoderTicks);
    joint.getData(sensedVelocity);
    joint.getData(sensedRoundsPerMinute);
    joint.getData(sensedCurrent);
    joint.getData(sensedTorque);
    std::stringstream angleSet, angleEncSet, velSet, velRPMSet, currentSet, pwmSet, torqueSet;

    switch (controllerMode) {
      case POSITION_CONTROL_RAD:
        angleSet << angleSetpoint.angle.value();
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case POSITION_CONTROL_ENC:
        angleSet << "NaN";
        angleEncSet << encoderSetpoint.encoderTicks;
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case VELOCITY_CONTROL_RAD_SEC:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << velocitySetpoint.angularVelocity.value();
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case VELOCITY_CONTROL_RPM:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << roundsPerMinuteSetpoint.rpm;
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case PWM_MODE:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << PWMSetpoint.pwm;
        torqueSet << "NaN";
        break;
      case CURRENT_MODE:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << currentSetpoint.current.value();
        pwmSet << "NaN";
        torqueSet << "NaN";
        break;
      case TORQUE_MODE:
        angleSet << "NaN";
        angleEncSet << "NaN";
        velSet << "NaN";
        velRPMSet << "NaN";
        currentSet << "NaN";
        pwmSet << "NaN";
        torqueSet << torqueSetpoint.torque.value();
        break;
    };



    file << timeDurationMicroSec //1
            << " " << angleSet.str() //2
            << " " << velSet.str() //3
            << " " << velRPMSet.str() //4
            << " " << currentSet.str() //5
            << " " << torqueSet.str() //6
            << " " << pwmSet.str() //7
            << " " << angleEncSet.str() //8

            << " " << sensedAngle.angle.value() //9
            << " " << sensedEncoderTicks.encoderTicks //10
            << " " << sensedVelocity.angularVelocity.value() //11
            << " " << sensedRoundsPerMinute.rpm //12
            << " " << sensedCurrent.current.value() //13
            << " " << sensedTorque.torque.value() //14

            << " " << bool(statusFlags & OVER_CURRENT) << " " //15
            << bool(statusFlags & UNDER_VOLTAGE) << " " //16
            << bool(statusFlags & OVER_VOLTAGE) << " " //17
            << bool(statusFlags & OVER_TEMPERATURE) << " " //18
            << bool(statusFlags & MOTOR_HALTED) << " " //19
            << bool(statusFlags & HALL_SENSOR_ERROR) << " " //20
            << bool(statusFlags & ENCODER_ERROR) << " " //21
            << bool(statusFlags & INITIALIZATION_ERROR) << " " //22
            << bool(statusFlags & PWM_MODE_ACTIVE) << " " //23
            << bool(statusFlags & VELOCITY_MODE) << " " //24
            << bool(statusFlags & POSITION_MODE) << " " //25
            << bool(statusFlags & TORQUE_MODE) << " " //26
            << bool(statusFlags & EMERGENCY_STOP) << " " //27
            << bool(statusFlags & FREERUNNING) << " " //28
            << bool(statusFlags & POSITION_REACHED) << " " //29
            << bool(statusFlags & INITIALIZED) << " " //30
            << bool(statusFlags & TIMEOUT) << " " //31
            << bool(statusFlags & I2T_EXCEEDED) << std::endl; //32


  // Bouml preserved body end 000C94F1
}


} // namespace youbot
