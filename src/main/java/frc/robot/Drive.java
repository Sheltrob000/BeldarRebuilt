package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Drive {

    private static final class Constants {

    }

    private final DriveSubsystem driveSubsystem;

    public Drive() {
        driveSubsystem = new DriveSubsystem();
    }

    public void addToSmartDashboard() {
        SmartDashboard.putData(driveSubsystem);
    }

    public void setDefaultCommand() {

    }

    public void addTestingcommandsToDashboard() {

    }

    private Supplier<ChassisSpeeds> createChassisSpeedsSupplier(
            CommandXboxController controller,
            double deadband,
            XboxController.Axis xAxis,
            XboxController.Axis yAxis) {
        return null;
    }

    public void bindFieldCentricCommandToController(
            CommandXboxController controller,
            XboxController.Axis xAxis,
            XboxController.Axis yAxis) {
        // TODO:

    }

    public void bindRobotCentricCommandToController(
            CommandXboxController controller,
            XboxController.Axis xAxis,
            XboxController.Axis yAxis) {
        // TODO:

    }

    public void bindRobotCentricCommandToController(){
        
    }

    public void bindModuleZeroControlToController(
            CommandXboxController controller,
            XboxController.Button button) {
        // TODO:

    }

    public void configureAutoBuilder(

    ) {

    }

}
