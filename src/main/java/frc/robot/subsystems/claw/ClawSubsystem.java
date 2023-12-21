package frc.robot.subsystems.claw;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

  public static enum State {
    OPEN,
    CLOSED
  }

  private static final class Constants {
    private static final PneumaticsModuleType ModuleType = PneumaticsModuleType.REVPH;
    private static final int moduleNumber = 17;
    private static final int leftChannel = 0;
    private static final int rightChannel = 1;
  }

  // fields
  Solenoid leftSolenoid;
  Solenoid rightSolenoid;

  // constructor
  public ClawSubsystem() {
    leftSolenoid = new Solenoid(Constants.moduleNumber, Constants.ModuleType, Constants.leftChannel);
    rightSolenoid = new Solenoid(Constants.moduleNumber, Constants.ModuleType, Constants.rightChannel);
  }

  // telemetry methods
  public State getState() {
    if(leftSolenoid.get() && rightSolenoid.get() == true){
      return State.OPEN;
    }else{
      return State.CLOSED;
    }
  }

  // control methods used by commands
  public void setState(State state) {
    if(state == State.OPEN){
      leftSolenoid.set(true);
      rightSolenoid.set(true);
    }else{
      leftSolenoid.set(false);
      rightSolenoid.set(false);
    }
  }

  // command creation methods. Note this only makes a command according
  // to the instructions in the method. It does not connect it to a trigger.
  public Command createSetStateCommand(State state) {
    Runnable clawSetCommandRunnable = () -> setState(state);
    Command clawSetCommand = runOnce(clawSetCommandRunnable);
    clawSetCommand.setName("Claw " + state.name());
    return clawSetCommand;
  }

  public Command createToggleCommand(){
    Command setOpenCommand = createSetStateCommand(getState());
    Command setClosedCommand = createSetStateCommand(getState());
    BooleanSupplier selector = () -> getState() == State.CLOSED;
    Command toggleCommand = Commands.either(setOpenCommand, setClosedCommand, selector);
    toggleCommand.setName("Toggle Command");
    return toggleCommand;
  }

  // Method runs stuff on subsystem that must change all of the time.
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // initSendable handles Dashboard
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Claw State", () -> getState().toString(), null);

  }
}
