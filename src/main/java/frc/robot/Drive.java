package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Drive {

    private final DriveSubsystem driveSubsystem;

    public Drive() {
        driveSubsystem = new DriveSubsystem();
    }

    public void addToSmartDashboard() {
        SmartDashboard.putData(driveSubsystem);
    }

    public void bindFieldCentricControlToController(
            CommandXboxController controller,
            XboxController.Axis xAxis,
            XboxController.Axis yAxis) {
        // TODO:

    }

    public void bindRobotCentricControlToController(
            CommandXboxController controller,
            XboxController.Axis xAxis,
            XboxController.Axis yAxis) {
        // TODO:

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
