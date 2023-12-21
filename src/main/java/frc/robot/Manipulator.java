package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystemReal;
import frc.robot.subsystems.grabber.GrabberSubsystemSim;

public class Manipulator {

    public static final class State {

        public State(

        ) {

        }
    }
    private final GrabberSubsystem grabberSubsystem;

    public Manipulator() {
        // TODO: ANGEL: initialize clawSubsystem to new ClawSubsystem();
        // TODO: KEITH: initialize tiltSubsystem to new TiltSubsystem();

        if(RobotBase.isSimulation()){
            grabberSubsystem = new GrabberSubsystemSim();
        }else{
            grabberSubsystem = new GrabberSubsystemReal();
        };
    }

    public void addToDashboard() {
        // TODO: ANGEL: use the SmartDashboard.putData() method to put clawSubsystem to
        // the Dashboard
        // TODO: KEITH: use the SmartDashboard.putData() method to put tiltSubsystem to
        // the Dashboard
        SmartDashboard.putData(grabberSubsystem);
    }

    public void setDefaultCommands() {
        grabberSubsystem.setDefaultCommand();
    }

    public void addMechanism2dWidget() {
        // TODO:
    }

    public void addTestingCommandsToDashboard() {
        // TODO: ANGEL create a Command called testOPENCommand using clawSubsystem's
        // createSetStateCommand()
        // TODO: ANGEL create a Command called testCLOSEDCommand using clawSubsystem's
        // createSetStateCommand()

        // TODO: KEITH create a Command called testFULLcommand using tiltSubsystem's
        // createSetStateCommand()
        // TODO: KEITH create a Command called testLONGcommand using tiltSubsystem's
        // createSetStateCommand()
        // TODO: KEITH create a Command called testSHORTcommand using tiltSubsystem's
        // createSetStateCommand()
        // TODO: KEITH create a Command called testNONEcommand using tiltSubsystem's
        // createSetStateCommand()


        Command testGrabber750RPMCommand = grabberSubsystem.createDriveAtVelocityCommand(750);
        Command testGrabber650RPMCommand = grabberSubsystem.createDriveAtVelocityCommand(650);
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

    public void bindTiltManualControlToControllerPOV(
            CommandXboxController controller) {
        // TODO: KEITH: create a Trigger called fullTrigger and initialize with
        // controller.povUp();
        // TODO: KEITH: create a Trigger called longTrigger and initialize with
        // controller.povUp();
        // TODO: KEITH: create a Trigger called shortTrigger and initialize with
        // controller.povRight();
        // TODO: KEITH: create a Trigger called NoneTrigger and initialize with
        // controller.povDown();
        // TODO: KEITH: create a Command called fullCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: create a Command called longCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: create a Command called shortCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: create a Command called noneCommand and initialize with the
        // createSetStateCommand from tiltSubsystem;
        // TODO: KEITH: bind the fullTrigger to the fullCommand with fullTrigger's
        // onTrue method

    }

    public void bindClawManualControlToController(
            CommandXboxController controller,
            XboxController.Button button) {
        // TODO: ANGEL: create a Command called toggleCommand using clawSubsystems
        // createToggleCommand method
        // TODO: ANGEL: create a Trigger called trigger and initialize to
        // controller.button(button.value)
        // TODO: ANGEL: bind the trigger to the toggleCommand using trigger's onTrue
        // method.

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
