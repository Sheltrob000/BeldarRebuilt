// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class WheelReal extends Wheel {

    private static final class Constants {
        private static final double kS = 0.10045;
        private static final MotorType motorType = MotorType.kBrushless;
    }

    CANSparkMax canSparkMax;
    
    public WheelReal(int moduleNumber) {
        super(Constants.kS);
        canSparkMax = new CANSparkMax(moduleNumber + 20,Constants.motorType);
    }

    @Override
    public double getPositionMeters() {
        double meters = canSparkMax.getEncoder().getPosition() * 2 * Math.PI * Wheel.Constants.wheelRadiusMeters / Wheel.Constants.gearing;
        return meters;
    }

    @Override
    public double getVelocityMetersPerSecond() {
        double metersPerSecond = canSparkMax.getEncoder().getVelocity() * 2 * Math.PI * Wheel.Constants.wheelRadiusMeters / 60 / Wheel.Constants.gearing;
        return metersPerSecond;
    }

    @Override
    public void setPositionMeters(double meters) {
        double rotations = meters * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters;
        canSparkMax.getEncoder().setPosition(rotations);
    }

    @Override
    public void setInputVoltage(double voltage) {
        canSparkMax.setVoltage(voltage);
    }
}
