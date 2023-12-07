// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;


public abstract class Wheel {

    protected static final class Constants {
        protected static final double dtSeconds = 0.020;
        protected static final double gearing = 6.75;
        protected static final double kV = 2.6158;
        protected static final double kA = 0.054006;
        private static final double maxVelocityErrorMetersPerSecond = 24.044;
        private static final LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        private static final Vector<N1> qelms = VecBuilder.fill(maxVelocityErrorMetersPerSecond);

        private static final Vector<N1> relms = VecBuilder.fill(RobotController.getBatteryVoltage());
        private static final LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(plant,
                qelms, relms, dtSeconds);
        private static final double kP = controller.getK().get(0, 0);
        private static final double kI = 0.0;
        private static final double kD = 0.0;
        private static final double wheelRadiusMeters = Units.inchesToMeters(4.0 / 2.0);
    }

    private final TrapezoidProfile trapezoidProfile;
    private final PIDController pidController;
    private final SimpleMotorFeedforward simpleMotorFeedforward;

    private double lastVelocity;

    public Wheel(double kS) {
        simpleMotorFeedforward = new SimpleMotorFeedforward(kS, Constants.kV, Constants.kA);
        double maxVelocity = simpleMotorFeedforward.maxAchievableVelocity(12, 0);
        double maxAcceleration = simpleMotorFeedforward.maxAchievableAcceleration(12, 0);
        Constraints constraints = new Constraints(maxVelocity, maxAcceleration);
        trapezoidProfile = new TrapezoidProfile(constraints);
        pidController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
        lastVelocity = 0.0;
    }

    public abstract double getPositionMeters();

    public abstract double getVelocityMetersPerSecond();

    public double getAccelerationMetersPerSecondSquared() {
        double current = getVelocityMetersPerSecond();
        return (current - lastVelocity) / Constants.dtSeconds;
    }

    public abstract void setPositionMeters(double meters);

    public void driveAtVelocity(double metersPerSecond) {
        double measurementVelocity = getVelocityMetersPerSecond();
        State goal = new State(metersPerSecond, 0.0);
        State current = new State(measurementVelocity, getAccelerationMetersPerSecondSquared());
        State achievableSetpoint = trapezoidProfile.calculate(measurementVelocity, goal, current);
        double feedbackVoltage = pidController.calculate(measurementVelocity, metersPerSecond);
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
