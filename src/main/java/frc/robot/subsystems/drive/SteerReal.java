package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class SteerReal extends Steer {

    private static final class Constants {
        private static final double kS = 0.12055;
        private static final MotorType motorType = MotorType.kBrushless;
    }

    CANSparkMax canSparkMax;

    public SteerReal(int moduleNumber) {
        super(Constants.kS);
        canSparkMax = new CANSparkMax(moduleNumber + 10, Constants.motorType);
    }

    @Override
    public double getPositionDegrees() {
        double degrees = canSparkMax.getEncoder().getPosition() * 360 / Steer.Constants.gearing;
        degrees = MathUtil.inputModulus(degrees, 0, 360);
        return degrees;
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        double degreesPerSecond = canSparkMax.getEncoder().getVelocity() * 360 / 60 / Steer.Constants.gearing;
        return degreesPerSecond;
    }

    @Override
    public void setPositionDegrees(double degrees) {
         degrees = MathUtil.inputModulus(degrees, 0, 360);
        double rotations = degrees * Steer.Constants.gearing / 360;
        canSparkMax.getEncoder().setPosition(rotations);
        
    }

    @Override
    public void setInputVoltage(double voltage) {
        canSparkMax.setVoltage(voltage);
    }
}
