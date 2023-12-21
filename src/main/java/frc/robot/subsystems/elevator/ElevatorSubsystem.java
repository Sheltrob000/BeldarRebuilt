// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import javax.swing.undo.StateEdit;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ElevatorSubsystem extends SubsystemBase {

        protected static class Constants {
                protected static final double dtSeconds = 0.020;
                protected static final int deviceId = 16;
                protected static final double gearing = 16;
                protected static final double drumRadiusMeters = Units.inchesToMeters(0.88);
                protected static final double kG = 0.26374;
                protected static final double kV = 14.346;
                protected static final double kA = 0.20482;
                protected static final double maxPositionErrorMeters = 0.125;
                protected static final double maxVelocityErrorMetersPerSec = 24.178;
                protected static final LinearSystem<N2, N1, N1> positionPlant = LinearSystemId.identifyPositionSystem(kV, kA);
                protected static final Vector<N2> positionQelms = VecBuilder.fill(maxPositionErrorMeters, maxVelocityErrorMetersPerSec);
                protected static final Vector<N1> positionRelms = VecBuilder.fill(RobotController.getBatteryVoltage());
                protected static final LinearQuadraticRegulator<N2, N1, N1> positionController = new LinearQuadraticRegulator<>(positionPlant, positionQelms, positionRelms, dtSeconds);
                protected static final LinearSystem<N1, N1, N1> velocityPlant = LinearSystemId.identifyVelocitySystem(kV, kA);
                protected static final Vector<N1> velocityQelsm = VecBuilder.fill(maxVelocityErrorMetersPerSec);
                protected static final Vector<N1> velocityRelms = VecBuilder.fill(RobotController.getBatteryVoltage());
                protected static final LinearQuadraticRegulator<N1, N1, N1> velocityController = new LinearQuadraticRegulator<N1, N1, N1>(velocityPlant, velocityQelsm, velocityRelms, dtSeconds);
                protected static final double kPPosition = positionController.getK().get(0,0);
                protected static final double kIPosition = 0.0;
                protected static final double kDPosition = positionController.getK().get(0,1);                
                protected static final double kPVelocity = velocityController.getK().get(0,0);
                protected static final double kIVelocity = 0.0;
                protected static final double kDVelocity = 0.0;
                protected static final double tolerance =  0.0239 * Math.PI * drumRadiusMeters / gearing;
        }

        TrapezoidProfile positionTrapezoidProfile;
        TrapezoidProfile velocityTrapezoidProfile;
        PIDController positionPIDController;
        PIDController velocityPIDController;
        ElevatorFeedforward elevatorFeedForward;

        double lastVelocity;

        public ElevatorSubsystem(double kS) {
                elevatorFeedForward = new ElevatorFeedforward(Constants.kA, Constants.kG, Constants.kV);
                double maxVelocity = elevatorFeedForward.maxAchievableVelocity(12, 0);
                double maxAcceleration = elevatorFeedForward.maxAchievableAcceleration(12, 0);
                Constraints positionConstraints = new Constraints(maxVelocity, maxAcceleration); 
                Constraints velocityConstraints = new Constraints(maxAcceleration, Double.POSITIVE_INFINITY);
                TrapezoidProfile positionTrapezoidProfile;
                TrapezoidProfile velocityTrapezoidProfile;
                PIDController positionPIDController;
                PIDController velocityPIDController;
                double lastVelocity = 0.0;
        }

        public abstract double getPositionMeters();

        public abstract double getVelocityMetersPerSecond();

        public double getAccelerationMetersPerSecondSquared() {

                double currentVelocityMetersPerSecond = getVelocityMetersPerSecond();
                return (currentVelocityMetersPerSecond - lastVelocity) / 0.020;
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
