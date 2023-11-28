package frc.robot.subsystems.tilt;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TiltSubsystem extends SubsystemBase {

  public enum State {
    FULL,
    LONG,
    SHORT,
    NONE
  }

  private static final class Constants {
    private static final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
    private static final int module = 17;
    private static final int shortChannel = 2;
    private static final int longChannel = 3;
  }

  // fields
  private final Solenoid shortSolenoid;
  private final Solenoid longSolenoid;

  // constructor
  public TiltSubsystem() {
    shortSolenoid = new Solenoid(Constants.module, Constants.moduleType, Constants.shortChannel);
    longSolenoid = new Solenoid(Constants.module, Constants.moduleType, Constants.longChannel);
  }

  // telemetry methods
  public State getState() {
    if (shortSolenoid.get() && longSolenoid.get()) {
      return State.FULL;
    } else if (!shortSolenoid.get() && longSolenoid.get()) {
      return State.LONG;
    } else if (shortSolenoid.get() && !longSolenoid.get()) {
      return State.SHORT;
    } else {
      return State.NONE;
    }
  }

  // control methods
  public void setState(State state) {
    if(state == State.FULL){
      shortSolenoid.set(true);
      longSolenoid.set(true);
    } else if(state == State.LONG){
      shortSolenoid.set(false);
      longSolenoid.set(true);
    }else if(state == State.SHORT){
      shortSolenoid.set(true);
      longSolenoid.set(false);
    } else {
      shortSolenoid.set(false);
      longSolenoid.set(false);
    }
  }

  // command creation methods
  public Command createSetStateCommand(State state) {
    Runnable tiltSet = () -> setState(state);
    Command tiltSetCommand = runOnce(tiltSet);
    tiltSetCommand.setName("Tilt " + state.name());
    return tiltSetCommand;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // initSendable handles Dashboard
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Tilt State", () -> getState().toString(), null);

  }
}
