package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;

public abstract class Steer {
    protected static final class Constants {
        private static final double dtSeconds = 0.020;
        private static final double gearing = -150.0 / 7.0;
        private static final double KV = 0.427607143;
        private static final double KA = 0.005493643;
        private static final double maxPositionErrorRadians = 0.125;
        private static final double maxVelocityErrorRadiansPerSec = 1;

        private static final LinearSystem<N2, N1, N1> plant = LinearSystemId.identifyPositionSystem(KV, KA);
        private static final Vector<N2> qelms = VecBuilder.fill(maxPositionErrorRadians, maxVelocityErrorRadiansPerSec);
        private static final Vector<N1> relms = VecBuilder.fill(RobotController.getBatteryVoltage());
        private static final LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(plant,
                qelms, relms, dtSeconds);

        private static final double KP = controller.getK().get(0, 0);
        private static final double KI = 0.0;
        private static final double kD = controller.getK().get(0, 1);
        private static final double tolerance = Math.abs(0.0239 * 2 * Math.PI / Constants.gearing);
    }

    private final TrapezoidProfile trapezoidProfile;
    private final PIDController pidController;
    private final SimpleMotorFeedforward simpleMotorFeedforward;

    public Steer(double kS) {
        simpleMotorFeedforward = new SimpleMotorFeedforward(kS, Constants.KV, Constants.KA);
        pidController = new PIDController(Constants.KP, Constants.KI, Constants.kD);
        pidController.setTolerance(Constants.tolerance);
        double maxVelocity = simpleMotorFeedforward.maxAchievableVelocity(12, 0);
        double maxAcceleration = simpleMotorFeedforward.maxAchievableAcceleration(12, 0);
        Constraints constraints = new Constraints(maxVelocity, maxAcceleration);
        trapezoidProfile = new TrapezoidProfile(constraints);
        pidController.setTolerance(Constants.maxPositionErrorRadians,Constants.maxVelocityErrorRadiansPerSec);
    }

    public abstract double getPositionDegrees();

    public abstract double getVelocityDegreesPerSecond();

    public abstract void setPositionDegrees(double degrees);

    public void turnToPosition(double degrees) {
        double measurementRadians = Math.toRadians(getPositionDegrees());
        double measurementRadiansPerSecond = Math.toRadians(getVelocityDegreesPerSecond());
        double setPointRadians = Math.toRadians(degrees);
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double difference = MathUtil.inputModulus(setPointRadians - measurementRadians, -errorBound, errorBound);
        setPointRadians = measurementRadians + difference;
        State goal = new State(setPointRadians, 0.0);
        State current = new State(measurementRadians, measurementRadiansPerSecond);
        State achievableSetpoint = trapezoidProfile.calculate(setPointRadians, goal, current);
        double feedbackVoltage = pidController.calculate(measurementRadians, setPointRadians);
        double feedforwardVoltage = simpleMotorFeedforward.calculate(measurementRadiansPerSecond, achievableSetpoint.velocity, Constants.dtSeconds);
        double voltage = feedbackVoltage + feedforwardVoltage;
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        setInputVoltage(voltage);
    }

    public abstract void setInputVoltage(double voltage);

}
