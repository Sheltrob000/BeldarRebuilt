// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.com.simulation.LinearWheelSim;
import frc.com.simulation.ModuleSteerSim;

public class WheelSim extends Wheel {

    private static final class Constants {
        private static final int numMotors = 1;
        private static final DCMotor dcMotor = DCMotor.getNEO(numMotors);
        private static final double kS = 0.0;
    }

    private final SimDouble simRotations;
    private final SimDouble simRPM;
    private final SimDouble simCurrent;
    private final SimDouble simVolts;
    private final LinearWheelSim wheelSim;

    public WheelSim(int moduleNumber) {
        super(Constants.kS);
        wheelSim = new LinearWheelSim(Wheel.Constants.kV, Wheel.Constants.kA, Constants.dcMotor);
        SimDevice simDevice = SimDevice.create("NEO", moduleNumber + 10);
        simRotations = simDevice.createDouble("Rotations", Direction.kBidir, 0.0);
        simRPM = simDevice.createDouble("RPM", Direction.kBidir, 0.0);
        simCurrent = simDevice.createDouble("Amps", Direction.kBidir, 0.0);
        simVolts = simDevice.createDouble("Volts", Direction.kBidir, 0.0);
    }

    @Override
    public double getPositionMeters() {
        return wheelSim.getPositionMeters();
    }

    @Override
    public double getVelocityMetersPerSecond() {
        return wheelSim.getVelocityMetersPerSecond();
    }

    @Override
    public void setPositionMeters(double meters) {
        wheelSim.setPositionMeters(meters);
    }

    @Override
    public void setInputVoltage(double voltage) {
        simVolts.set(voltage);
        wheelSim.setInputVoltage(voltage);
        simRotations.set(wheelSim.getPositionMeters() * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters);
        simRPM.set(wheelSim.getVelocityMetersPerSecond() * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters);
        simCurrent.set(wheelSim.getCurrentDrawAmps());
        wheelSim.update(Wheel.Constants.dtSeconds);
    }
}
