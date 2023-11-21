// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

public abstract class Wheel {

    protected static final class Constants {
        // TODO: create a double called dtSeconds and initialize to 0.020
        // TODO: create a double called gearing and initialize to 6.75
        // TODO: create a double called kV and initialize to 2.6158
        // TODO: create a double called kA and initialize to 0.054006
        // TODO: create a double called maxVelocityErrorMetersPerSecond and initialize
        // to 24.044
        // TODO: create a LinearSystem<N1, N1, N1> called plant and initialize to
        // LinearSystemId.identifyVelocitySystem(kV, kA)
        // TODO: create a Vector<N1> called qelms and initialize to
        // VecBuilder.fill(maxVelocityErrorsMetersPerSecond)
        // TODO: create a Vector<N1> called relms and initialize to
        // VecBuilder.fill(RobotController.getBatteryVoltage())
        // TODO: creat a LinearQuadraticRegulator<N1, N1, N1> called controller and
        // initialize with previous constants
        // TODO: create a double called kP and initialize with controller.getK().get(0,
        // 0)
        // TODO: create a double called kI and initialize to 0.0
        // TODO: create a double called kD and initialize to 0.0
        // TODO: create a double called wheelRadiusMeters and initialize to
        // Units.inchesToMeters(4.0 / 2.0)
    }

    // TODO: create a field of type TrapezoidProfile called trapezoidProfile
    // TODO: create a field of type PIDController called pidController
    // TODO: create a field of type SimpleMotorFeedforward called
    // simpleMotorFeedforward
    // TODO: create a field of type double called lastVelocity

    public Wheel(double kS) {
        // TODO: initialize simpleMotorFeedforward with approrpriate constants
        // TODO: create a double called maxVelocity and initialize to
        // simpleMotorFeedforward.maxachievableVelocity(12, 0)
        // TODO: create a double called maxAcceleration and intialize to
        // simpleMotorFeedforward.maxAchievableAcceleration(12, 0)
        // TODO: create a Constraints object called contraints and intialize with
        // maxVelocity and maxAcceleration
        // TODO: initialize trapezoidProfile with appropriate constants
        // TODO: initialize pidController with approrpiate constants
        // TODO: initialize lastVelocity to 0.0
    }

    public abstract double getPositionMeters();

    public abstract double getVelocityMetersPerSecond();

    public double getAccelerationMetersPerSecondSquared() {
        // TODO: create a double called current and initialize to
        // getVelocityMetersPerSecond()
        // TODO: return (current - lastVelocity) / Constants.dtSeconds
        return 0.0; // TODO: remove this line when done
    }

    public abstract void setPositionMeters(double meters);

    public void driveAtVelocity(double metersPerSecond) {
        // TODO: create a double called measurementVelocity and initiliaze to
        // getVelocityMetersPerSecond()
        // TODO: create a State called goal and initialize with metersPerSeconda and 0.0
        // TODO: create a State called current and initialize with measurementVelocity
        // and getAccelerationsMetersPerSecondSquared()
        // TODO: creat a State called achievableSetpoint and get from
        // trapezoidProfile.calculate
        // TODO: create a double called feedbackVoltage and intialize using
        // pidController's calculate method
        // TODO: create a double called feedforwardVoltage and initialize wiht
        // simpleMotorFeedforward.calculate(measurementVelocity,
        // achievableSetpoint.position, dtSeconds)
        // TODO: create a double called voltage as the sum of the two previous voltages
        // TODO: clamp voltage using MathUtil.clamp(voltage, -12.0, 12.0);
        // TODO: setInputVoltage to voltage
        // TODO: set lastVelocity to measurementVelocity
    }

    public abstract void setInputVoltage(double voltage);
}
