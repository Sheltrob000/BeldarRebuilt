// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.system.plant.DCMotor;
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
    private final ModuleSteerSim weelSim;

    public WheelSim(int moduleNumber) {
        super(Constants.kS);
        weelSim = new ModuleSteerSim(Wheel.Constants.kV, Wheel.Constants.kA, Constants.dcMotor);
        SimDevice simDevice = SimDevice.create("NEO", moduleNumber + 10);
        // TODO: initialize simRotations with simDevice.createDouble("Rotations", Direction.kBidir, 0.0)
        // TODO: intialize simRPM with simDevice.createDouble("RPM", Direction.kBidir, 0.0)
        // TODO: initialize simCurrent with simDevice.createDouble("Amps", Direction.kBidir, 0.0)
        // TODO: initialize simVolts with simDevice.createDouble("Volts", Direction.kBidir, 0.0)
    }

    @Override
    public double getPositionMeters() {
        // TODO: return getPositonMeters() from wheelSim
        return 0.0; // TODO: remove this line when done.
    }

    @Override
    public double getVelocityMetersPerSecond() {
        // TODO: return getVelocityMetersPerSecond() from wheelSim
        return 0.0; // TODO: remove this line when done.
    }

    @Override
    public void setPositionMeters(double meters) {
        // TODO: setPositionMeters for wheelSim
    }

    @Override
    public void setInputVoltage(double voltage) {
        // TODO: set simVolts to voltage
        // TODO: setInputVoltage for wheelSim
        // TODO: set simRotations with wheelSim.getPositionMeters() * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters
        // TODO: set simRPM with wheelSim.getVelocityMetersPerSecond() * Wheel.Constants.gearing / 2 / Math.PI / Wheel.Constants.wheelRadiusMeters
        // TODO: set simCurrent with wheelSim.getCurrentDrawAmps()
        // TODO: update wheelSim with dtSeconds
    }
}
