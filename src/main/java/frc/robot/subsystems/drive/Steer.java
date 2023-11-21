package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;

public abstract class Steer {
    protected static final class Constants {
        private static final double dtSeconds = 0.020;
        private static final double gearing = -150.0 / 7.0;
        private static final double KV = 0.427607143;
        private static final double KA = 0.005493643;
        private static final double maxPositionErrorRadians = 0.125;
        private static final double maxVelocityErrorRadiansPerSec = 1;

        LinearSystem<N2, N1, N1>plant = LinearSystemId.identifyPositionSystem(KV, KA);
        Vector<N2>qelms = VecBuilder.fill(maxPositionErrorRadians, maxVelocityErrorRadiansPerSec);
        Vector<N1>relms = VecBuilder.fill(RobotController.getBatteryVoltage());
        LinearQuadraticRegulator<N2, N1, N1>controller = new LinearQuadraticRegulator<>(plant, qelms, relms, dtSeconds);

        private final double KP = controller.getK().get(0, 0);
        private final double KI = 0.0;
        private final double kD = controller.getK().get(0, 1);
        private final double tolerance = Math.abs(0.0239 * 2 * Math.PI / Constants.gearing);
    }


    private final TrapezoidProfile trapezoidProfile;
    private final PIDController pidController;
    private final SimpleMotorFeedforward simpleMotorFeedforward;

    public Steer(double kS) {
        // TODO: initialize simpleMotorFeedforward with appropriate constants
        // TODO: initialize pidController with appropriate constants
        // TODO: create a double called maxVelocity and initialize to simpleMotorFeedforward.maxachievableVelocity(12, 0)
        // TODO: create a double called maxAcceleration and intialize to simpleMotorFeedforward.maxAchievableAcceleration(12, 0)
        // TODO: create a Constraints object called contraints and intialize with maxVelocity and maxAcceleration
        // TODO: initialize trapezoidProfile with appropriate constants
        // TODO: setTolerance for the pidController with appropriate constants
    }

    public abstract double getPositionDegrees();

    public abstract double getVelocityDegreesPerSecond();

    public abstract void setPositionDegrees(double degrees);

    public void turnToPosition(double degrees) {
        // TODO: create a double called measurementRadians and initialize to Math.toRadians(getPositionDegrees())
        // TODO: create a double called measurementRadiansPerSecond and initialize to Math.toRadians(getVelocityDegreesPerSecond())
        // TODO: create a double called setPointRadians and initialize to Math.toRadians(degrees)
        // TODO: create a double called errorBound and initialize to (pi - -pi) / 2.0
        // TODO: create a double called difference and intialize MathUtil.inputModulus(setpointRadians - measurementRadians, -errorBound, errorBound)
        // TODO: reassign setpointRadian to measurementRadians + difference;
        // TODO: create a State called goal and initialize to setpointRadians and 0.0
        // TODO: create a State called current and initialize to measurementRadians and measurementRadiansPerSecond
        // TODO: create a State called achievableSetpoint and initliaze from trapezoidProfile.calculate
        // TODO: create a double called feedbackVoltage and calculate from pidController
        // TODO: create a double called feedforwardVoltage and calculate from simpleMotorFeedforward.calculate(measurementRadiansPerSecond, achievableSetpoint.velocity, dtSeconds)
        // TODO: create a double called voltage and initialize to the sum of the two previous voltages
        // TODO: voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        // TODO: setInputVoltage to voltage
    }
    
    public abstract void setInputVoltage(double voltage);

}
