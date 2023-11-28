package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemReal;
import frc.robot.subsystems.arm.ArmSubsystemSim;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystemReal;
import frc.robot.subsystems.elevator.ElevatorSubsystemSim;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystemReal;
import frc.robot.subsystems.grabber.GrabberSubsystemSim;
import frc.robot.subsystems.tilt.TiltSubsystem;

public class Manipulator {

    public static final class State {

        public State(

        ) {

        }
    }

    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final GrabberSubsystem grabberSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final TiltSubsystem tiltSubsystem;

    public Manipulator() {
        armSubsystem = RobotBase.isSimulation()
                ? new ArmSubsystemSim()
                : new ArmSubsystemReal();
        elevatorSubsystem = RobotBase.isSimulation()
                ? new ElevatorSubsystemSim()
                : new ElevatorSubsystemReal();
        grabberSubsystem = RobotBase.isSimulation()
                ? new GrabberSubsystemSim()
                : new GrabberSubsystemReal();
        clawSubsystem = new ClawSubsystem();
        tiltSubsystem = new TiltSubsystem();
    }

    public void addToDashboard() {
        SmartDashboard.putData(armSubsystem);
        SmartDashboard.putData(elevatorSubsystem);
        SmartDashboard.putData(grabberSubsystem);
        SmartDashboard.putData(clawSubsystem);
        SmartDashboard.putData(tiltSubsystem);
    }

    public void setDefaultCommands() {
        armSubsystem.setDefaultCommand();
        elevatorSubsystem.setDefaultCommand();
        grabberSubsystem.setDefaultCommand();
    }

    public void addMechanism2dWidget() {
        // TODO:
    }

    public void addTestingCommandsToDashboard(){

    }

    public void bindArmManualControlToController(
            CommandXboxController controller,
            XboxController.Axis axis,
            double deadband) {
        // TODO:

    }

    public void bindElevatorManualControlToController(
            CommandXboxController controller,
            XboxController.Axis axis,
            double deadband) {
        // TODO:

    }

    public void bindGrabberManualControlToController(
            CommandXboxController controller,
            double deadband) {
        // TODO:

    }

    public void bindTiltManualControlToController(
            CommandXboxController controller) {
        // TODO:

    }

    public void bindClawManualControlToController(
            CommandXboxController controller) {
        // TODO:

    }

    public void bindManipulatorStateToController(
            CommandXboxController controller,
            XboxController.Button button) {
        // TODO:

    }

    public void addManipulatorStateToNamedCommands(
            String stateName,
            Manipulator.State state) {
        // TODO:

    }

}
