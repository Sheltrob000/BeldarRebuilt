package frc.robot.subsystems.drive;

public abstract class Steer {
    protected static final class Constants {
        private static final double dtSeconds = 0.020;
        private static final double gearing = -150.0 / 7.0;
        private static final double KV = 0.427607143;
        private static final double KA = 0.005493643;
        private static final double maxPositionErrorRadians = 0.125;
        private static final double maxVelocityErrorRadiansPerSec = 1;
        // TODO: create a LinearSystem<N2, N1, N1> called plant and initialize to
        // LinearSystemId.identifyPositionSystem(kV, kA)


        // TODO: create a Vector<N2> called qelms and initialize to
        // VecBuilder.fill(maxPositionErrorRadians, maxVelocityErrorRadiansPerSec)
        
        // TODO: create a Vector<N1> called relms and initialize to
        // VecBuilder.fill(RobotController.getBatteryVoltage())

        // TODO: create a LinearQuadraticRegular<N2, N1, N1> called controller and
        // initialize with appropriate constants

        // TODO: create a double called kP initialize to controller.getK().get(0, 0)

        // TODO: create a double called kI and initialize to 0.0

        // TODO: create a double called kD and initialize to controller.getK().get(0, 1)
        
        // TODO: create a double called tolerance and initialize to Math.abs(0.0239 * 2
        // * Math.PI / Constants.gearing)
    }

    // TODO: create a field of type TrapezoidProfile called trapezoidProfile
    // TODO: create a field of type PIDController called pidController
    // TODO: create a field of type SimpleMotorFeedforward called

    public Steer(double kS) {
        // TODO: initialize simpleMotorFeedforward with appropriate constants
        // TODO: initialize pidController with appropriate constants
        // TODO: initialize trapezoidProfile with appropriate constants
        // TODO: setTolerance for the pidController with appropriate constants
    }

    public abstract double getPositionDegrees();

    public abstract double getVelocityDegreesPerSecond();

    public abstract void setPositionDegrees(double degrees);

    public void turnToPosition(double degrees) {
        // TODO: create a double called measurementRadians and initialize to Math.toRadians(getPositionDegrees())
        // TODO: create a double called measurementRadiansPerSecond and initialize to Math.toRadians(getVelocityDegreesPerSecond())
        // TODO: create a double called setPointRadians and initialize to wrapSetpointAngle(measurementRadians, degrees)
        // TODO: create a State called goal and initialize to setpointRadians and 0.0
        // TODO: create a State called current and initialize to measurementRadians and measurementRadiansPerSecond
        // TODO: create a State called achievableSetpoint and initliaze from trapezoidProfile.calculate
        // TODO: create a double called feedbackVoltage and calculate from pidController
        // TODO: create a double called feedforwardVoltage and calculate from simpleMotorFeedforward.calculate(measurementRadiansPerSecond, achievableSetpoint.velocity, dtSeconds)
        // TODO: create a double called voltage and initialize to the sum of the two previous voltages
        // TODO: voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        // TODO: setInputVoltage to voltage
    }

    public double wrapSetpointAngle(double measurementRadians, double degrees) {
        // TODO: create a Rotation2d called measurement2d and initialize to Rotation2d.fromRadians(measurementRadians)
        // TODO: create a Rotation2d called setPoint2d and initialize to Rotation2d.fromDegrees(MathUtil.inputModulus(degrees, 0, 360));
        // TODO: create a Rotation2d called difference2d and initialize to setPoint2d.minus(measurement2d);
        // TODO: return measurement2d.plus(difference2d).getRadians();
        return 0.0; //TODO: remove this line when finished
    }

    
    public abstract void setInputVoltage(double voltage);

}
