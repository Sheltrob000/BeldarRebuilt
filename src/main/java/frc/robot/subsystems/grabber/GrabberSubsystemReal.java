package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class GrabberSubsystemReal extends GrabberSubsystem {

  private static final class Constants {
    private static final double kS = 0.23971;
    private static final MotorType motorType = MotorType.kBrushless;
  }

  // fields
  private CANSparkMax leftGripperCanSparkMax;
  private CANSparkMax rightGripperCanSparkMax;

  // constructor
  public GrabberSubsystemReal() {
    super(Constants.kS);
    leftGripperCanSparkMax = new CANSparkMax(GrabberSubsystem.Constants.leftGripperDeviceId,Constants.motorType);
    rightGripperCanSparkMax = new CANSparkMax(GrabberSubsystem.Constants.rightGripperDeviceId,Constants.motorType);
  }

  // telemetry methods
  @Override
  public double getLeftGripperVelocityRPM() {
    return leftGripperCanSparkMax.getEncoder().getVelocity() * GrabberSubsystem.Constants.gearing;
  }

  @Override
  public double getLeftGripperVelocityRadPerSec() {
    return Units.rotationsPerMinuteToRadiansPerSecond(getLeftGripperVelocityRPM());
  }

  @Override
  public double getRightGripperVelocityRPM() {
    return rightGripperCanSparkMax.getEncoder().getVelocity() * GrabberSubsystem.Constants.gearing;
  }

  @Override
  public double getRightGripperVelocityRadPerSec() {
    return Units.rotationsPerMinuteToRadiansPerSecond(getRightGripperVelocityRPM());
  }

  @Override
  public void setLeftGripperInputVoltage(double voltage) {
    leftGripperCanSparkMax.setVoltage(voltage);
  }

  @Override
  public void setRightGripperInputVoltage(double voltage) {
    rightGripperCanSparkMax.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
