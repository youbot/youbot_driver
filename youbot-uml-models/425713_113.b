class DataTrace
!!!823153.cpp!!!	DataTrace(inout youBotJoint : YouBotJoint)

    roundsPerMinuteSetpoint.rpm = 0;
    PWMSetpoint.pwm = 0;
    encoderSetpoint.encoderTicks = 0;
!!!824305.cpp!!!	startTrace() : void
    

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
    
    traceStartTime = microsec_clock::local_time();
!!!824433.cpp!!!	stopTrace() : void

    file.close();

!!!823409.cpp!!!	updateTrace(in setpoint : JointAngleSetpoint) : void
    angleSetpoint = setpoint;
    this->update();
!!!823537.cpp!!!	updateTrace(in setpoint : JointVelocitySetpoint) : void
    velocitySetpoint = setpoint;
    this->update();
!!!823665.cpp!!!	updateTrace(in setpoint : JointRoundsPerMinuteSetpoint) : void
    roundsPerMinuteSetpoint = setpoint;
    this->update();
!!!823793.cpp!!!	updateTrace(in setpoint : JointCurrentSetpoint) : void
    currentSetpoint = setpoint;
    this->update();
!!!823921.cpp!!!	updateTrace(in setpoint : JointTorqueSetpoint) : void
    torqueSetpoint = setpoint;
    this->update();
!!!824049.cpp!!!	updateTrace(in setpoint : JointPWMSetpoint) : void
    PWMSetpoint = setpoint;
    this->update();
!!!824177.cpp!!!	updateTrace(in setpoint : JointEncoderSetpoint) : void
    encoderSetpoint = setpoint;
    this->update();
!!!824561.cpp!!!	update() : void
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

      file << timeDurationMicroSec //1
            << " " << angleSetpoint.angle.value() //2
            << " " << velocitySetpoint.angularVelocity.value() //3
            << " " << roundsPerMinuteSetpoint.rpm  //4
            << " " << currentSetpoint.current.value() //5
            << " " << torqueSetpoint.torque.value()  //6
            << " " << PWMSetpoint.pwm  //7
            << " " << encoderSetpoint.encoderTicks  //8
            
            << " " << sensedAngle.angle.value() //9
            << " " << sensedEncoderTicks.encoderTicks   //10
            << " " << sensedVelocity.angularVelocity.value()   //11
            << " " << sensedRoundsPerMinute.rpm   //12
            << " " << sensedCurrent.current.value()   //13
            << " " << sensedTorque.torque.value()   //14
            
            << " " << bool(statusFlags & OVER_CURRENT) << " "   //15
            << bool(statusFlags & UNDER_VOLTAGE) << " "   //16
            << bool(statusFlags & OVER_VOLTAGE) << " "   //17
            << bool(statusFlags & OVER_TEMPERATURE) << " "   //18
            << bool(statusFlags & MOTOR_HALTED) << " "   //19
            << bool(statusFlags & HALL_SENSOR_ERROR) << " "   //20
            << bool(statusFlags & ENCODER_ERROR) << " "   //21
            << bool(statusFlags & INITIALIZATION_ERROR) << " "   //22
            << bool(statusFlags & PWM_MODE_ACTIVE) << " "   //23
            << bool(statusFlags & VELOCITY_MODE) << " "   //24
            << bool(statusFlags & POSITION_MODE) << " "   //25
            << bool(statusFlags & TORQUE_MODE) << " "   //26
            << bool(statusFlags & EMERGENCY_STOP) << " "   //27
            << bool(statusFlags & FREERUNNING) << " "   //28
            << bool(statusFlags & POSITION_REACHED) << " "   //29
            << bool(statusFlags & INITIALIZED) << " "   //30
            << bool(statusFlags & TIMEOUT) << " "   //31
            << bool(statusFlags & I2T_EXCEEDED) << std::endl;


!!!824689.cpp!!!	plotTrace() : void
  system("gnuplot ../gnuplotconfig");
