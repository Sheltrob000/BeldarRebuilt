package frc.com.sensors.absoluteAngle;

import java.util.function.DoubleSupplier;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.com.simulator.Simulatable;
import frc.com.simulator.Simulator;

public class AbsAngleEncoderSim implements AbsAngleEncoder, Simulatable {

    private final DoubleSupplier angleDegreesSupplier;
    private final SimDouble simAbsoluteAngle;

    public AbsAngleEncoderSim(DoubleSupplier angleDegreesSupplier, String name, int index) {
        this.angleDegreesSupplier = angleDegreesSupplier;
        SimDevice simDevice = SimDevice.create(name, index);
        simAbsoluteAngle = simDevice.createDouble("Absolute Angle Degrees", Direction.kBidir, 0.0);
        Simulator.getInstance().addToSimulator(this);
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(simAbsoluteAngle.get());
    }

    @Override
    public void update() {
        double angleDegrees = angleDegreesSupplier.getAsDouble();
        angleDegrees = angleDegrees < 0 ? angleDegrees + 360 : angleDegrees;
        simAbsoluteAngle.set(angleDegrees);
    }

    @Override
    public boolean isDataGood() {
        return true;
    }

}
