// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ElevatorSubsystem extends SubsystemBase {

        protected static class Constants {
                protected static final double dtSeconds = 0.020;
                // TODO: create an int called deviceId and set equal to 16
                // TODO: create a double called grearing and set equal to 16
                // TODO: create a double called drumRadiusMeters and set to
                // Units.inchesToMeters(0.88)
                // TODO: create a double called kG and set equal to 0.26374
                // TODO: create a double called kV and set equal to 14.346
                // TODO: create a double called kA and set equal to 0.20482
                // TODO: create a double called maxPositionErrorMeters and set equal to 0.125
                // TODO: create a double called maxVelocityErrorMetersPerSec and set equal to
                // 24.178
                // TODO: create a LinearSystem<N2, N1, N1> called positionPlant and initialize
                // with LinearSystemId.identifyPositionSystem
                // TODO: create a Vector<N2> called positionQelms and initialize with
                // VecBuilder.fill(maxPositionErrorMeters, maxVelocityErrorMetersPerSec)
                // TODO: create a Vector<N1> called positionRelms and initialize with
                // VecBuilder.fill(RobotController.getBatteryVoltage())
                // TODO: create a LinearQuadraticRegular<N2, N1, N1> called positionController
                // and initialize with appropriate values
                // TODO: create a LinearSystem<N1, N1, N1> called velocityPlant and initialize
                // with LinearSystemId.identifyVelocitySystem
                // TODO: create a Vector<N1> called velocityQelsm and initialize with
                // VecBuilder.fill(maxVelocityErrorMetersPerSec)
                // TODO: create a LinearQuadraticRegular<N1, N1, N1> called velocityController
                // and initialize with LinearQuadraticRegulator<N1, N1, N1> velocityController
                // with appropriate constants
                // TODO: create a double called kPPosition and initialize to
                // positionController.getK().get(0, 0)
                // TODO: create a double called kIPosition and set equal to 0.0
                // TODO: create a double called kDPosition and initialize to
                // positionController.getK().get(0, 1)
                // TODO: create a double called kPVelocity and initialize to
                // velocityController.getK().get(0, 0)
                // TODO: create a double called kIVelocity and set equal to 0.0
                // TODO: create a double called kDVelocity and set equal to 0.0
                // TODO: create a double called tolerance and set equal to 0.0239 * pi *
                // drumRadiusMeters / gearing
        }

        // TODO: create a TrapezoidProfile called positionTrapezoidProfile
        // TODO: create a TrapezoidProfile called velocityTrapezoidProfile
        // TODO: create a PIDController called positionPIDController
        // TODO: create a PIDController called velocityPIDController
        // TODO: create an ElevatorFeedforward called elevatorFeedForward

        // TODO: create a double called lastVelocity

        public ElevatorSubsystem(double kS) {
                // TODO: initialize elevatorFeedForward with appropriate constants
                // TODO: create a double called maxVelocity and initialize to
                // elevatorFeedForward.maxachievableVelocity(12, 0)
                // TODO: create a double called maxAcceleration and intialize to
                // elevatorFeedForward.maxAchievableAcceleration(12, 0)
                // TODO: create a Constraints object called positionConstraints and intialize with
                // maxVelocity and maxAcceleration
                // TODO: create a Constraints object called velocityContraints and intialize with maxAcceleration and Double.PositiveInfinity
                // TODO: initialize positionTrapezoidProfile
                // TODO: initialize velocityTrapezoidProfile
                // TODO: initialize positionPIDController
                // TODO: intialize velocityPIDController
                // TODO: initialize lastVelocity to 0.0
        }

        public abstract double getPositionMeters();

        public abstract double getVelocityMetersPerSecond();

        public double getAccelerationMetersPerSecondSquared() {
                // TODO: create a double called currentVelocityMetersPerSecond and set equal to
                // getVelocityMetersPerSecond();
                // TODO: return (currentVelocityMetersPerSecond - lastVelocity) / 0.020
                return 0.0; // TODO: remove this line when finished
        }

        public abstract void setPositionMeters(double meters);

        public void moveToPosition(double meters) {
                // TODO: create a double called measurement get from getPositionMeters()
                // TODO: create a double called measurementVelocity get from
                // getVelocityMetersPerSecond()
                // TODO: create a State called current get from measurement and
                // measurementVelocity
                // TODO: create a State called goal get from meters and 0.0 for velocity
                // TODO: create a State called achievableSetpoint and calculate from
                // positionTrapezoidProfile
                // TODO: create a double called feedbackVoltage and calculate from
                // positionPIDController, measurement and achievableSetpoint.position
                // TODO: create a double called feedforwardVoltage and calculate from
                // elevatorFeedforward, measurementVelocity, achievableSetpoint.velocity, and
                // dtSeconds
                // TODO: create a double called voltage as the sum of the two previous voltages
                // TODO: voltage = MathUtil.clamp(voltage, -12.0, 12.0);
                // TODO: setInputVoltage to voltage
        }

        public void driveAtVelocity(double metersPerSecond) {
                // TODO: create a double called measurementVelocity get from
                // getVelocityMetersPerSecond()
                // TODO: create a State called current get from measurementVelocity and
                // getAccelerationMetersPerSecondSquared()
                // TODO: create a State called goal get from metersPerSecond and 0.0 for
                // velocity
                // TODO: create a State called achievableSetpoint and calculate from
                // velocityTrapezoidProfile
                // TODO: create a double called feedbackVoltage and calculate from
                // velocityPIDController, measurementVelocity and achievableSetpoint.position
                // TODO: create a double called feedforwardVoltage and calculate from
                // elevatorFeedforward, measurementVelocity, achievableSetpoint.position, and
                // dtSeconds
                // TODO: create a double called voltage as the sum of the two previous voltages
                // TODO: voltage = MathUtil.clamp(voltage, -12.0, 12.0);
                // TODO: setInputVoltage to voltage
        }

        public abstract void setInputVoltage(double voltage);

        public Command createMoveToPositionCommand(double meters) {
                // TODO: create a Runnable called resetControllers and set to () ->
                // positionPIDController.reset()
                // TODO: create a Command called resetControllersCommand and initialize with
                // runOnce(resetControllers)
                // TODO: create a Runnable called moveToPosition and set to () ->
                // moveToPosition(meters)
                // TODO: create a Command called moveToPositionCommand and initialize with
                // run(moveToPosition)
                // TODO: create a BooleanSupplier called endingCondition and set to () ->
                // positionPIDController.atSetpoint()
                // TODO: create a Runnable called endingCleanupt and set to () ->
                // driveAtVelocity(0.0)
                // TODO: create a Command called command and set to
                // resetControllersCommand
                // .andThen(moveToPositionCommand)
                // .until(endingCondition)
                // .finallyDo(endingCleanup)
                // TODO: setName for command to String.format("%s meters Command", meters)
                // TODO: return command
                return null; // TODO: remove this line when done
        }

        public Command createDriveAtVelocityCommand(double metersPerSecond) {
                // TODO: create a Runnable called resetControllers and set to () ->
                // velocityPIDControllers.reset()
                // TODO: create a Command called resetControllersCommand initialize with
                // runOnce(resetControllers)
                // TODO: create a Runnable called driveAtVelocity and set to () ->
                // driveAtVelocity(metersPerSecond)
                // TODO: create a Command called driveAtVelocityCommand and initialize with
                // run(driveAtVelocity)
                // TODO: create a Runnable called endingCleanup and set to () ->
                // driveAtVelocity(0.0)
                // TODO: creat a Command called called and set to
                // resetControllersCommand
                // .andThen(driveAtVelocityCommand)
                // .finallyDo(endingCleanup)
                // TODO: setName for command to String.format("%s MPS Command", metersPerSecond)
                // TODO: return command
                return null; // TODO: remove this line when done
        }

        public void setDefaultCommand() {
                // TODO: create a Runnable called defaultRunnable and set to () ->
                // driveAtvelocity(0.0)
                // TODO: create a Command called defaultCommand and initialize with
                // run(defaultRunnable)
                // TODO: setName for defaultCommand to String.format("Stop Command")
                // TODO: setDefaultCommand(defaultCommand);
        }

        // Method runs stuff on subsystem that must change all of the time.
        @Override
        public void periodic() {
                // TODO: set lastVelocityMetersPerSecond to getVelocityMetersPerSecond()
        }

        // Method handles reporting to the smartdashboard
        @Override
        public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);
                builder.addDoubleProperty("Elevator Position (Meters)", () -> getPositionMeters(), null);
                builder.addDoubleProperty("Elevator Speed (MPS)", () -> getVelocityMetersPerSecond(), null);
                builder.addDoubleProperty("Elevator Acceleration (MPS_Sq)",
                                () -> getAccelerationMetersPerSecondSquared(), null);
        }

}
