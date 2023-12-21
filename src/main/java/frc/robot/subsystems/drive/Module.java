package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

public class Module {

    private static final class Constants {
        // TODO: create a String called absAngleEncoderCanBus and intialize to
        // "CANIVORE"
    }

    private final Steer steer;
    // TODO: Make a field for the Wheel object named wheel
    // TODO: Make a field for the AbsAngleEncoder called absAngEncoder
    // TODO: make a field for moduleNumber

    public Module(int moduleNumber) {
        steer = RobotBase.isSimulation()
                ? new SteerSim(moduleNumber)
                : new SteerReal(moduleNumber);
        // TODO: initialize the wheel in a similar fashion to the steer
        // TODO: FREEBEE initialize absAngEncoder just comment out
        // absAngleEncoder = RobotBase.isSimulation()
        // ? new AbsAngleEncoderSim(
        // () -> steer.getPositionDegrees(),
        // "CANCoder",
        // moduleNumber + 30)
        // : new AbsAngleEncoderCanCoder(
        // moduleNumber + 30,
        // Constants.absAngleEncoderCanBus);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        // TODO: create a double called positionMeters and initialize from wheel's
        // getPositionMeters() method
        // TODO: create a double called positionDegrees and initialize from steer's
        // getPositionDegrees() method
        // TODO: create a Rotation2d object called angle and intialize using
        // Rotation2d's static fromDegrees() method.
        // TODO: create a SwerveModulePosition object called swerveModulePosition and
        // intialize using positionMeters and angle
        // TODO: return swerveModulePosition
        return null; // TODO: remove this line when done.
    }

    public SwerveModuleState getSwerveModuleState() {
        // TODO: create a double called velocityMetersPerSecond and initialize from
        // wheel's
        // getVelocityMetersPerSecond() method
        // TODO: create a double called positionDegrees and initialize from steer's
        // getPositionDegrees() method
        // TODO: create a Rotation2d object called angle and intialize using
        // Rotation2d's static fromDegrees() method.
        // TODO: create a SwerveModuleState object called swerveDriveState and
        // intialize using velocityMetersPerSecond and angle
        // TODO: return SwerveModuleState
        return null; // TODO: remove this line when done.
    }

    public double getAbsoluteAngleEncoderDegrees() {
        // return absoluteAngles angle in degrees
        return 0.0; // TODO: remove this line when done.
    }

    public boolean isAbsoluteEncoderDataGood() {
        // this is here to DriveSubsystem can put this on the dashboard for testing.
        // return absAngleEncoder's isDataGood method result
        return false; // TODO: return this line when done.
    }

    public void setSwerveModuleAngle(double degrees) {
        // TODO: FREEBEE. Just comment out.
        // basically this checks if absoluteEncoder data is good
        // sets it to it if so.
        // else sets its to 0 assuming we manually turned the module.
        // if (isAbsoluteEncoderDataGood()) {
        // double degrees = getAbsoluteAngleEncoderDegrees();
        // steer.setPositionDegrees(degrees);
        // } else {
        // System.out.println("Bad AbsoluteEncoderData for Module " + moduleNumber + " :
        // Assume manually set to 0.0");
        // steer.setPositionDegrees(0.0);
        // }
    }

    public void controlModule(double degrees, double metersPerSecond) {
        // TODO: turn the module using steer's turnToPosition method passing in degrees
        // TODO: drive the wheel using the wheel's driveAtVelocity method passing in
        // metersPerSecond
    }

    public void controlModule(double degrees) {
        // TODO: NOTE this method is just a convenience feature.
        // TODO: Call controlModule pass in the degrees parameter and 0.0;
    }

    public void stop() {
        // TODO: NOTE another convenience feature. Applies zero voltage to both the
        // steer and the wheel
        // TODO: use steer's setInputVoltage and set to 0.0
        // TODO: use the wheel's setInputVoltage and set to 0.0
    }
}
