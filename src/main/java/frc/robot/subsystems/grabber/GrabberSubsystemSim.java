package frc.robot.subsystems.grabber;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.com.simulation.FlywheelSim;

public class GrabberSubsystemSim extends GrabberSubsystem {

  private static final class Constants {
    private final static int numMotorsPerGripper = 1;
    private final static DCMotor dcmotor = DCMotor.getNeo550(numMotorsPerGripper);
    private static final double kS = 0.0;
  }

  // fields
  SimDouble leftSimRotations;
  SimDouble leftSimRPM;
  SimDouble leftSimCurrent;
  SimDouble leftSimVolts;
  SimDouble rightSimrotations;
  SimDouble rightSimRPM;
  SimDouble rightSimCurrent;
  SimDouble rightSimVolts;
  FlywheelSim leftGripperSim;
  FlywheelSim rightGripperSim;

  // constructor
  public GrabberSubsystemSim() {
    super(Constants.kS);
    SimDevice leftSimDevice = SimDevice.create("NEO550", GrabberSubsystem.Constants.leftGripperDeviceId);
    SimDevice rightSimDevice = SimDevice.create("NEO550",  GrabberSubsystem.Constants.rightGripperDeviceId);
    leftSimRotations = leftSimDevice.createDouble("Rotations",Direction.kBidir, 0.0);
    leftSimRPM = leftSimDevice.createDouble("RPM", Direction.kBidir, 0.0);
    leftSimCurrent = leftSimDevice.createDouble("Amps", Direction.kBidir, 0.0);
    leftSimVolts = leftSimDevice.createDouble("volts", Direction.kBidir, 0.0);
    rightSimrotations = rightSimDevice.createDouble("Rotations", Direction.kBidir, 0.0);
    rightSimRPM = rightSimDevice.createDouble("RPM", Direction.kBidir , 0.0);
    rightSimCurrent = rightSimDevice.createDouble("Amps", Direction.kBidir, 0.0);
    rightSimVolts = rightSimDevice.createDouble("Volts", Direction.kBidir, 0.0);

    leftGripperSim = new FlywheelSim(
      GrabberSubsystem.Constants.kV, 
      GrabberSubsystem.Constants.kA, 
      Constants.dcmotor);

    rightGripperSim = new FlywheelSim(
      GrabberSubsystem.Constants.kV, 
      GrabberSubsystem.Constants.kA, 
      Constants.dcmotor);
  }

  // telemetry methods
  @Override
  public double getLeftGripperVelocityRPM() {
    return leftGripperSim.getAngularVelocityRPM();
  }

  @Override
  public double getLeftGripperVelocityRadPerSec() {
    return leftGripperSim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getRightGripperVelocityRPM() {
    return rightGripperSim.getAngularVelocityRPM();
  }

  @Override
  public double getRightGripperVelocityRadPerSec() {
    return rightGripperSim.getAngularVelocityRPM();
  }

  // control methods
  @Override
  public void setLeftGripperInputVoltage(double voltage) {
    leftSimVolts.set(voltage);
    leftGripperSim.setInputVoltage(voltage);
  }

  @Override
  public void setRightGripperInputVoltage(double voltage) {
    rightSimVolts.set(voltage);
    rightGripperSim.setInputVoltage(voltage);
  }

  @Override
  public void periodic() {
    leftGripperSim.update(GrabberSubsystem.Constants.dtSeconds);
    rightGripperSim.update(GrabberSubsystem.Constants.dtSeconds);

  leftSimRotations.set(leftGripperSim.getAngularPositionRad() * GrabberSubsystem.Constants.gearing / 2 / Math.PI);
  leftSimRPM.set(getLeftGripperVelocityRPM() * GrabberSubsystem.Constants.gearing);
  leftSimCurrent.set(leftGripperSim.getCurrentDrawAmps());
  rightSimrotations.set(rightGripperSim.getAngularPositionRad() * GrabberSubsystem.Constants.gearing / 2 / Math.PI);
  rightSimRPM.set(getRightGripperVelocityRPM() * GrabberSubsystem.Constants.gearing);
  rightSimCurrent.set(rightGripperSim.getCurrentDrawAmps());
  }
}
