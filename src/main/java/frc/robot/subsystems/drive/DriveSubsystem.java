package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // NOTES: The DriveSubsystem is responsible for moving the robot and for knowing
    // where it is on the field

    public static final class Constants {

        // DriveSubsystem is public. Everything with access to this file has access to
        // this class

        // Constants is an inner class. You can define a class for use inside of
        // another. Or if they share a common theme.

        // The outer DriveSubsystem class has access to everything inside the inner
        // classes, but we make them private so that
        // items with access to the DriveSubsystem from outside can't directly access
        // constants unless we want them to.
        // if we wanted child classes of DriveSubsystem to access them we'd make them
        // protected.
        // if we wanted everything to have access to them we make them public

        // Every variable in Constants is made final, because they are constants.

        // final variables are constants
        // final methods can't be overriden in subclasses
        // final classes can't have subclasses

        // static means you don't have to create an object of this type to use the inner
        // class, methods, or variables.
        // static stuff is access with className.thingName

        // Constants is public. Everything with access to the DriveSubsystem class has
        // access to Constants

        // Constants is static. It comes into existence the minute the class is loaded
        // into memory
        // You don't have to create a DriveSubsystem variable with the new keyword and
        // the constructors to use Constants

        // Constants is final. A final class means no subclasses can be made from it.

        // You can access it in the file with Constants.
        // You can access it outside of the file with DriveSubsystem.Constants

        // TODO: Note all items in this inner class should be private static final
        // except
        // where noted.

        // TODO: create an int called numModules and set to 4

        // TODO: create a double called robotLengthMeters and initialize to the result
        // of Units.inchesToMeters() for 30 inches

        // TODO: create a double called robotWidthMeters and initialize to the result of
        // Units.inchesToMeters() for 24 inches

        // TODO: create a double called xModuleMag and initialize to robotLengthMeters /
        // 2

        // TODO: create a double called yModuleMage and intialize to robotWidthMeters /
        // 2

        // TODO: create a double called driveBaseRadius. This is calculated as the
        // distance from the robot to each modules.
        // YOU will use pythagorean theorem to determine this using xModuleMag and
        // yModuleMag
        // This will be public static final

        // TODO: create a Translation2d called frontLeftPosition and initialize using
        // the constructor that takes an x coordinate and a y coordinate
        // the x coordinate is xModuleMag and the y coordinate is yModuleMag

        // TODO: repeat the previous TODO for frontRightPosition, rearLeftPosition, and
        // rearRightPosition
        // note that you'll use the same values for x and y, but different signs.
        // +x is forward,
        // +y is left

        // TODO: create a SwerveDriveKinematics called swerveDriveKinematics and
        // intialize
        // the constructor that takes in a Translation2d... remember varargs!
        // just feed in frontLeftPosition, frontRightPosition, etc... into the
        // constructor

        // TODO: create a Pose2d called initialPoseMeters and initialize to the
        // constructor that takes in x, y, and angle
        // set x to 3, y to 3 and angle tor Rotation2d.fromDegrees(0);
        // this will be used to give the robot an initial position other than the bottom
        // corner of the field
        // Field coordinate system.
        // +x is the driver looking to the opposite alliance's field.
        // +y is driver looking towards their left
        // + degrees rotates to the driver's left. or 0 to 180 degrees
        // - degrees rotates to the driver's right. or 360 to 180 degrees

        // TODO: go to your Steer and Wheel classes and make the gearings public and
        // wheelRadiusMeters
        // TODO: this one's public. create a double called maxVelocityMetersPerSecond.
        // Now we would use sysId to find kV for the drive train while driving straight.
        // Then use the kV, kA, and kS constants in a simpleMotorFeedforward to find the
        // maxVelocity and maxAcceleration
        // Max velocity should pretty much be 12.0 / Wheel.Constants.kV
        // Max acceleration could be done with module kA and not driveTrain kA, but it
        // will be too big.
        // But we'll use that until we can test the driveTrain (and figure out how to
        // get data)
        // and in case your wondering we need these numbers for autos. And yes we need
        // to do this for rotation as well.
        // TODO: create a public double called maxVelocityMetersPerSecond and intialize
        // to
        // 12.0 / (Wheel.Constants.kV)

        // TODO: create a public double called maxVelocityMetersPerSecond and intialize
        // to
        // 12.0 / (Wheel.Constants.kA)

        // TODO: create a public double called rotationalVelocityPercOfMax and set to 1
        // Robot maxRotation speed is really fast. We'll have to scale it down later.

        // TODO: create a public double called maxVelocityRadiansPerSecond
        // this should not require us to use sysId. The robots moment of inertia will
        // alter it's acceleration
        // initialize to rotationalVelocityPercOfMax * maxVelocityMetersPerSecond /
        // driveBaseRadius
        // PHYSICS !!!

        // TODO: create a public double called maxAccelerationRadiansPerSecondSquared
        // this won't be accurate until we use sysId(and figure out how to do so with
        // swerve)
        // initialize to rotationalVelocityPercOfMax *
        // maxAccelerationMetersPerSecondSquared / driveBaseRadius
    }

    // fields are private and final. They exist when an object of type
    // DriveSubsystem is created.
    // The reason we made all of those constants static is because we only need one
    // copy of them
    // if we made them non-static we would get extra copies that we don't need.
    // Of course we're only going to make one drivetrain, but that maybe not true
    // for other
    // subsystems. If we had a robot with two elevators that were identical, we'd
    // not want two copies of everything.
    // My personal preference is to declare fields but intialize them in the
    // constructor.
    // Mostly because of horizontal space concerns, but also some times we have to
    // do a lot
    // to intialize the field variables.

    // TODO: create a SwerveDrivePoseEstimate called swerveDrivePoseEstimator;
    // TODO: create a Module called frontLeftModule
    // TODO: create a Module called frontRightModule
    // TODO: create a Module called rearLeftModule
    // TODO: create a Module called rearRightModule
    // TODO: create a Gyroscope called gyro
    // TODO: create a Field2d called field2d

    public DriveSubsystem() {
        // TODO: intialize frontLeftModule to new Module(0);
        // TODO: repeat previous TODO for remaining modules, 1, 2, 3

        // TODO: if RobotBase.isSimulations then gyro = new GyroScopeSim
        // that will take a () -> getWheelStates() and name NavXMXP2
        // else gyro = GyroscopeNavX

        // TODO: initialize swerveDrivePoseEstimator
        // its arguments are the swerveDriveKinematics we made in Constants
        // the gyro angle so gyro.getGyroAngle()
        // and array of SwerveModulePositions. like below
        // new SwerveModulePosition[] {
        // frontLeftModule.getSwerveModulePosition(),
        // frontRightModule.getSwerveModulePosition(),
        // rearLeftModule.getSwerveModulePosition(),
        // rearRightModule.getSwerveModulePosition(),
        // }
        // and finally the initialPoseMeters we created in Constants

        // TODO: intialize field2d
        // TODO: setRobotPose for field2d to initialPoseMeters
        // TODO: use the putData method to send field2d to the dashboard.
    }

    public Pose2d getPosition() {
        // TODO: return getEstimatedPosition() from swerveDrivePoseEstimator
        return null; // TODO: remove this when done.
    }

    private SwerveDriveWheelPositions getWheelPositions() {
        // TODO: create a SwerveModulePosition called frontLeftModulePosition and
        // initialize with frontLeftModule's getSwerveModulePosition() method
        // TODO: repeat for 3 remaining modules
        // TODO: create a SwerveModulePosition[] called swerveModulePositions and
        // initialize like this
        // = new SwerveModulePosition[]{
        // frontLeftModulePosition,
        // frontRightModulePosition,
        // rearLeftModulePosition,
        // rearRightModulePosition
        // };
        // TODO: return a new SwerveDriveWheelPositions taking swerveModulePositions as
        // its argument
        return null; // TODO: remove this when done.
    }

    private SwerveDriveWheelStates getWheelStates() {
        // TODO: second verse same as the first.
        // TODO Replace the word Position with State
        return null; // TODO: remove this when done.

    }

    public ChassisSpeeds getRobotCentricChassisSpeeds() {
        // TODO: create a SwerveDriveWheelStates variable called swerveDriveWheelStates
        // and initialize to
        // getWheelStates()
        // TODO: use swerveDriveKinematics's tochassisSpeeds method with the
        // swerveDriveWheelStates as the argument to return the robotCentricChassiSpeeds
        return null; // TODO: remove this when done.
    }

    public ChassisSpeeds getFieldCentricChassisSpeeds() {
        // TODO: create a ChassisSpeeds called robotCentricChassisSpeeds and get from
        // the getRobotCentricChassisSpeeds method
        // TODO: create a Rotation2d called gyroAngle get from the getGyroAngle method
        // of gyro
        // TODO: return the result from ChassisSpeeds.fromRobotRelativeSpeeds taking in
        // the two previous variables as arguments. BTW new function in the beta here. I
        // did not make it though :(
        // but I offered a fix
        return null; // TODO: remove this when done.
    }

    // TODO: done with telemetry methods onto control methods
    // NOTE: These will all be void functions.

    public void setPosition(Pose2d poseMeters) {
        // TODO: create a Rotation2d variable called gyroAngle intialize using gyro's
        // getGyroAngle methods

        // TODO: create a SwerveDriveWheelPositions variable called
        // swerveDriveWheelPositions and initialize with the getWheelPositions method we
        // made

        // TODO: call swerveDrivePoseEstimator's resetPosition method with arguments
        // being
        // gyroAngle, swerveDriveWheelPositions, and the passed in poseMeters
    }

    public void setModuleAngles() {
        // TODO: this is a method that handles setting the module angles
        // to the values given to it by the CANCoders (AbsAngleEncoders)

        // TODO: call the setSwerveModuleAngle method for frontLeftModule
        // repeat for remaining modules
        // DONE!
    }

    public void turnModules(
            double frontLeftModuleDegrees,
            double frontRightModuleDegrees,
            double rearLeftModuleDegrees,
            double rearRightModuleDegrees) {
        // TODO: use frontLeftModule's controlModule method with frontLeftModuleDegrees
        // TODO: repeat for remaining modules
        // TODO: NOTE: this method allows us to do things like zero the modules once
        // we setup a command for it.
    }

    public void turnModules(double moduleDegrees) {
        // TODO: this is an override the uses the previous turnModules method to turn
        // all of them to the same position
        // TODO: I think you can handle this one. One line of code :)
    }

    public void controlModules(SwerveDriveWheelStates swerveDriveWheelStates) {
        // TODO: create a SwerveModuleState called frontLeftModuleState and get from
        // swerveDriveWheelStates.states[0]
        // TODO: repeat for remaining modules

        // TODO: call frontLeftModule's controlModule method and pass in as arguments
        // frontLeftModuleState.angle.getDegrees()
        // frontLeftModuleState.speedMetersPerSecond
        // TODO: repeate for reamining modules
    }

    public void driveAtRobotCentricChassisSpeeds(ChassisSpeeds robotCentricChassisSpeeds) {
        // TODO: create a SwerveDriveWheelStates variable called swerveDriveWheelStates
        // and get from
        // swerveDriveKinematics' toWheelSpeeds method taking in
        // robotCentricChassisSpeeds as its arguments
        // TODO: create a SwerveModuleState called frontLeftModuleState and get from
        // swerveDriveWheelStates.states[0]
        // TODO: repeat for remaining modules

        // TODO: create a SwerveDriveWheelStates variable called
        // currentSwerveDriveWheelStates and get from
        // the getWheelStates methods we created
        // TODO: create a SwerveModuleState called currentFrontLeftModuleState and get
        // from currentSwerveDriveWheelStates.states[0]
        // TODO: repeat for remaining modules

        // TODO: create a Rotation2d variable called currentFrontLeftModuleAngle and get
        // from currentFrontLeftModuleState.angle
        // TODO: repeat for remaining modules

        // TODO: create a SwerveModuleState variable called newFrontLeftModuleState and
        // get from SwerveModuleState.optimize method passing in frontLeftState and
        // currentFrontLeftAngle
        // TODO: repeat for remaining modules

        // TODO: create a SwerveModuleState[] called newSwerveModuleStates and intialize
        // like this
        // = new SwerveModuleState[] {
        // newFrontLeftModuleState,
        // newFrontRightModuleState,
        // newRearLeftModuleState,
        // newRearRightModuleState
        // };

        // TODO: create a SwerveDriveWheelStates variable called newWheelStates and
        // initialize with newSwerveModuleStates

        // TODO: call controlModules method passing in newWheelStates
        // NOTE: that was the longest method in this class file.
    }

    public void driveAtFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricChassisSpeeds) {
        // TODO: create a Pose2d variable called robotPosition and get from the
        // getPosition method we made
        // TODO: create a Rotation2d variable called robotAngle and get from the
        // robotPosition's getRotation method
        // TODO: create a ChassisSpeeds variable called robotCentricChassisSpeeds and
        // get from the ChassisSpeeds.fromFieldRelativeSpeeds method passing in
        // fieldCentricChassisSpeeds and robotAngle
        // TODO: call the driveAtRobotCentricChassisSpeeds method passing in
        // robotCentricChassisSpeeds
        // NOTE: see how fieldCentric depends on robotCentric.
        // NOTE: in programming special cases are often easier to write than the general
        // case,
        // but not always.
    }

    public void stopModules() {
        // TODO: you've seen me do this before. Call the stop method on all four
        // modules.
        // It's the littlest method that could.
    }

    // TODO: now we're going to make a bunch of command creator methods.
    // TODO: these will use the control methods above along with help from the
    // telemetry methods
    // to create commands that we can bind to buttons, register with Pathplanner, or
    // add to the dashboard

    public Command createSetEncodersFromAbsCommand() {
        // TODO: this tells the modules to reset their motor encoders to the angle
        // provided
        // by the CANCoders (AbsAngleEncoders)
        // TODO: create a Runnable called setEncoderFromAbs and initialize to () ->
        // setModuleAngles();
        // TODO: create a Command called setEncodersFromAbsCommand and initialize to
        // runOnce(setEncodersFromAbs)
        // TODO: setName for the command you just made to "Set Encoders From Absolute"
        // TODO: return setEncodersFromAbsCommand
        /// NOTE: SubsystemBase gives you run and runOnce that you can call (cause they
        // exist in SubsystemBase the parent class) to make this process easier. These
        // are essentially calling
        // Commands.run and Command.runOnce but saves on typing.
        // these decorators take a runnable. That's why we make the Runnable's first.
        // And to make things cleaner and easier to debug we put the runnable's (or
        // functions we can easily make to runnable)
        // and define them as control methods.
        return null; // TODO: remove this when done.
    }

    public Command createResetPositionCommand(Pose2d newPosition) {
        // TODO: this resets the data stored in swerveDrivePoseEstimator. We have a
        // control method
        // that does this called setPosition. Buts its a consumer not a runnable.
        // well create Runnable called resetPosition and set to () ->
        // setPosition(newPosition)
        // we made a runnable by putting a call to a consumer inside the runnable's
        // block of code.
        // neat trick :)
        // TODO: create a Command called resetPositionCommand and intialize to
        // runOnce(resetPosition)
        // TODO: use resetPositionCommand's setName method to set the name to the
        // following
        // String.format("Reset Position to %s m, %s m , %s degrees",
        // newPosition.getX(),
        // newPosition.getY(),
        // newPosition.getRotation().getDegrees()));
        return null; // TODO: remove this when done.
    }

    public Command createFieldCentricDriveCommand(Supplier<ChassisSpeeds> chassisSpeedsSupplier, String name) {
        // TODO: note the arguments. This command creator will be used when binding our
        // controller input to the drive. The controller supplies the chassisSpeeds so
        // we have to do some
        // work to make our runnable
        // Remember before
        // Runnable driveFieldCentric = () ->
        // driveFieldCentricChassisSpeeds(chassisSpeedsSupplier.get());
        // this runnable run's by asking a consumer to consume a value supplied by a
        // supplier.
        // yes I wrote that sentence.
        // TODO: createe a Command called driveFieldCentricCommand. Now this runs
        // forever so set to
        // run(driveFieldCentric). runOnce and run require Runnable's as arguments.
        // TODO: setName to name for the command
        // TODO: return the command.
        return null; // TODO: remove this when done.
    }

    public Command createFieldCentricDriveCommand(ChassisSpeeds chassisSpeeds) {
        // TODO: Note: this method is an overload of the previous, but useful for
        // testing
        // and for auto's that don't use pathplanner. The argument isn't a supplier.
        // but we'll make one and pass to the above method.
        // We also have to craft a name.

        // TODO: We'll make a Supplier<ChassisSpeeds> that will supply the value
        // chassisSpeeds forever
        // () -> chassisSpeeds

        // TODO: String name = String.format(
        // "Field Centric VX:%s, VY:%s, OMEGA:%s",
        // chassisSpeeds.vxMetersPerSecond,
        // chassisSpeeds.vyMetersPerSecond,
        // chassisSpeeds.omegaRadiansPerSecond);

        // TODO: now return the result of createFieldCentricDriveCommand passing in the
        // chassisSpeedsSupplier you just made
        // and the name
        return null; // TODO: remove this when done.
    }

    public Command createRobotCentricDriveCommand(
            Supplier<ChassisSpeeds> chassisSpeedsSupplier, String name) {
        // TODO: identical to field centric version. just with field replacing robot
        return null; // TODO: remove this when done.

    }

    public Command createRobotCentricDriveCommand(ChassisSpeeds chassisSpeeds) {
        // TODO: identical to field centric version. just with field replacing robot
        return null; // TODO: remove this when done.
    }

    public void setDefaultCommand() {
        // TODO: this simplifies the setDefaultCommand method for this subsystem by
        // making an overload
        // TODO: make a runnable called defaultRunnable and initialize to () ->
        // stopModules()
        // the little method that could. :)
        // I wanted to call this default, but that's a reserved keyword in Java
        // TODO: create a Command called defaultCommand and set to run(defaultRunnable)
        // default Commands run forever. Our default is to do nothing. When no other
        // command is running the robot will stop
        // When we don't move the joysticks it also stops. But this command will rarely
        // execute in teleop
        // Because either fieldCentric or robotCentric commands are running once we tie
        // them to the joystick.
        // It will run in Auto, depending on how stingy pathplanner is with the drive,
        // but as long as pathplanner works we're fine.
        // TODO: finally call the subsystems' native setDefaultCommand to default
        // command but be careful don't call the one with no arguments that's the one we
        // just made
    }

    // TODO: finally we're to the point where we need to handle updates to telemetry
    // from external encoders and gyros. We'll make a method called update() that
    // periodic will call
    // every 20 ms.

    public void update() {
        // TODO: create a Rotation2d called gyroAngle get from gyro's getGyroAngle
        // method
        // TODO: create a SwerveDriveWheelPositions variable called
        // swerveDriveWheelStates and get
        // from the getWheelPositions method we made
        // called swerveDrivePoseEstimator's update function taking in the two previous
        // variables
        // TODO: create a Pose2d called robotPosition and get from our getPosition
        // method
        // setRobotPose for the field2d field and pass in robotPosition (Updates the
        // map)
    }

    @Override
    public void periodic() {
        super.periodic();
        // TODO: Every 20 ms this method runs. It's used to update things like telemetry
        // from external encoders
        // We could ask the CANcoders to do this like in 2021, but we've learned some
        // things since then.
        // We do however need to update the swerveDrivePoseEstimator to things like the
        // limelight and gyro.
        // We'll do the limelight later.
        // Note, the arm and elevator could also use this method with their absolute
        // encoder's, but their flaky,
        // but we could write an update method for them too taking in some sort of
        // averaging protocol.
        // TODO: updatePosition()
        // FREEBEE: This makes the drive stop in simulation when disables.
        // I might move this later.
        // if (!RobotController.isSysActive()) {
        // stopModules();
        // }
    }

    // initSendable handles Dashboard
    // comment me out please :)
    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     super.initSendable(builder);
    //     builder.addBooleanArrayProperty("IsAbsAngleEncoderDataGood",
    //             () -> {
    //                 return new boolean[] {
    //                         frontLeftModule.isAbsoluteEncoderDataGood(),
    //                         frontRightModule.isAbsoluteEncoderDataGood(),
    //                         rearLeftModule.isAbsoluteEncoderDataGood(),
    //                         rearRightModule.isAbsoluteEncoderDataGood() };
    //             }, null);
    //     builder.addDoubleArrayProperty("AbsAngleEncoderDegreesData",
    //             () -> {
    //                 return new double[] {
    //                         frontLeftModule.getAbsoluteAngleEncoderDegrees(),
    //                         frontRightModule.getAbsoluteAngleEncoderDegrees(),
    //                         rearLeftModule.getAbsoluteAngleEncoderDegrees(),
    //                         rearRightModule.getAbsoluteAngleEncoderDegrees()
    //                 };
    //             }, null);
    //     builder.addDoubleProperty("Gyro Angle Degrees", () -> gyro.getGyroAngle().getDegrees(), null);
    //     builder.addDoubleProperty("MaxLinearVelocity", () -> Constants.maxVelocityMetersPerSecond, null);
    //     builder.addDoubleProperty("MaxRotationalVelocity", () -> Constants.maxVelocityRadiansPerSecond, null);
    //     builder.addDoubleProperty("MaxLinearAcceleration", () -> Constants.maxAccelerationMetersPerSecond, null);
    //     builder.addDoubleProperty("MaxRotationalAcceleration", () -> Constants.maxAccelerationRadiansPerSecond, null);
    //     builder.addDoubleArrayProperty(
    //             "Field Centric Speeds",
    //             () -> new double[] {
    //                     getFieldCentricChassisSpeeds().vxMetersPerSecond,
    //                     getFieldCentricChassisSpeeds().vyMetersPerSecond,
    //                     getFieldCentricChassisSpeeds().omegaRadiansPerSecond
    //             },
    //             null);
    //     builder.addDoubleArrayProperty(
    //             "Robot Centric Speeds",
    //             () -> new double[] {
    //                     getRobotCentricChassisSpeeds().vxMetersPerSecond,
    //                     getRobotCentricChassisSpeeds().vyMetersPerSecond,
    //                     getRobotCentricChassisSpeeds().omegaRadiansPerSecond
    //             },
    //             null);
    //     builder.addDoubleArrayProperty(
    //             "Field Position",
    //             () -> new double[] {
    //                     getPosition().getX(),
    //                     getPosition().getY(),
    //                     MathUtil.inputModulus(getPosition().getRotation().getDegrees(), -180, 180)
    //             },
    //             null);
    // }

    // TODO:  DONE with the subsystem.  
    // TODO: now to bind commands to triggers :)

}
