package frc.robot.subsystems.drive;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.com.simulation.ModuleSteerSim;

public class SteerSim extends Steer {

    private static final class Constants {
        private static final double kS = 0.0;
        private static final int numMotors = 1;
        private static final DCMotor dcMotor = DCMotor.getNEO(numMotors);
    }

    private final SimDouble simRotations;
    private final SimDouble simRPM;
    private final SimDouble simCurrent;
    private final SimDouble simVolts;
    private final ModuleSteerSim steerSim;

    public SteerSim(int moduleNumber) {
        super(Constants.kS);
        steerSim = new ModuleSteerSim(Steer.Constants.KV, Steer.Constants.KA, Constants.dcMotor);
        SimDevice simDevice = SimDevice.create("NEO", moduleNumber + 10);
        simRotations = simDevice.createDouble("Rotations", Direction.kBidir, 0.0);
        simRPM = simDevice.createDouble("RPM", Direction.kBidir, 0.0);
        simCurrent = simDevice.createDouble("Amps", Direction.kBidir, 0.0);
        simVolts = simDevice.createDouble("Volts", Direction.kBidir, 0.0);
    }

    @Override
    public double getPositionDegrees() {
        return steerSim.getPositionDegrees();
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        return steerSim.getVelocityDegreesPerSecond();
    }

    @Override
    public void setPositionDegrees(double degrees) {
        steerSim.setPositionDegrees(degrees);
    }

    @Override
    public void setInputVoltage(double voltage) {
        simVolts.set(voltage);
        simRotations.set(steerSim.getPositionRadians() * Steer.Constants.gearing / 2 / Math.PI);
        simRPM.set(steerSim.getVelocityRadiansPerSecond() * Steer.Constants.gearing * 60.0 / 2 / Math.PI);
        simCurrent.set(steerSim.getCurrentDrawAmps());
        steerSim.setInputVoltage(voltage);
        steerSim.update(Steer.Constants.dtSeconds);
    }
}
