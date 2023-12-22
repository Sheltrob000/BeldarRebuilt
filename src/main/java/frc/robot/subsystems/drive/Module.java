package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.com.sensors.absoluteAngle.AbsAngleEncoder;
import frc.com.sensors.absoluteAngle.AbsAngleEncoderCanCoder;
import frc.com.sensors.absoluteAngle.AbsAngleEncoderSim;
import frc.robot.Robot;

public class Module {

    private static final class Constants {
        private static final String absAngleEncoderCanBus = "CANIVORE";
    }

    private final Steer steer;
    private final Wheel wheel;
    private final AbsAngleEncoder absAngleEncoder;
    private final int moduleNumber;

    public Module(int moduleNumber) {
        this.moduleNumber = moduleNumber;
        steer = RobotBase.isSimulation()
                ? new SteerSim(moduleNumber)
                : new SteerReal(moduleNumber);
        wheel = RobotBase.isSimulation()
                ? new WheelSim(moduleNumber)
                : new WheelReal(moduleNumber);

        absAngleEncoder = RobotBase.isSimulation()
                ? new AbsAngleEncoderSim(
                        () -> steer.getPositionDegrees(),
                        "CANCoder",
                        moduleNumber + 30)
                : new AbsAngleEncoderCanCoder(
                        moduleNumber + 30,
                        Constants.absAngleEncoderCanBus);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double positionMeters = wheel.getPositionMeters();
        double positionDegrees = steer.getPositionDegrees();
        Rotation2d angle = Rotation2d.fromDegrees(positionDegrees);
        SwerveModulePosition swerveModulePosition = new SwerveModulePosition(positionMeters, angle);
        return swerveModulePosition;
    }

    public SwerveModuleState getSwerveModuleState() {
        double velocityMetersPerSecond = wheel.getVelocityMetersPerSecond();
        double positionDegrees = steer.getPositionDegrees();
        Rotation2d angle = Rotation2d.fromDegrees(positionDegrees);
        SwerveModuleState swerveModuleState = new SwerveModuleState(velocityMetersPerSecond, angle);
        return swerveModuleState;
    }

    public double getAbsoluteAngleEncoderDegrees() {
        // return absoluteAngles angle in degrees
        return absAngleEncoder.getAbsoluteAngle().getDegrees();
    }

    public boolean isAbsoluteEncoderDataGood() {
        return absAngleEncoder.isDataGood();
    }

    public void setSwerveModuleAngle() {
        if (isAbsoluteEncoderDataGood()) {
        double degrees = getAbsoluteAngleEncoderDegrees();
        steer.setPositionDegrees(degrees);
        } else {
        System.out.println("Bad AbsoluteEncoderData for Module " + moduleNumber + " : Assume manually set to 0.0");
        steer.setPositionDegrees(0.0);
        }
    }

    public void controlModule(double degrees, double metersPerSecond) {
        steer.setPositionDegrees(degrees);
        wheel.driveAtVelocity(metersPerSecond);
    }

    public void controlModule(double degrees) {
        controlModule(degrees, 0.0);
    }

    public void stop() {
        steer.setInputVoltage(0.0);
        wheel.setInputVoltage(0.0);
    }
}
