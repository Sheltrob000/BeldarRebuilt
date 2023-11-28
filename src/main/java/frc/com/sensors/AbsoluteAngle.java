package frc.com.sensors;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteAngle {

    public static final Supplier<Rotation2d> createAbsoluteAngleSupplier(
            CANCoder cancoder) {
        return () -> {
            double angleDegrees = cancoder.getAbsolutePosition();
            angleDegrees = angleDegrees < 0 ? angleDegrees + 360 : angleDegrees;
            return Rotation2d.fromDegrees(angleDegrees);
        };
    }

    public static final Supplier<Rotation2d> createAbsoluteAngleSupplier(
            Supplier<Double> angleDegreesSupplier, String name, int index) {
        // TODO: create a SimDevice called simDevice using the SimDevice.create(name,
        // index) method
        // TODO: create a SimDouble called simDegrees and intialize to
        // simDevice.createDouble(), using "Absolute Degrees" for the name,
        // Direction.kBidir for the direction and 0.0 for the intial value
        return () -> {
            // TODO: create a double called angleDegrees and intialize from the
            // angleDegreesSupplier get() method
            // TODO: adjust angleDegrees like I did i the above example
            // TODO: set simDegrees to angleDegrees using the set() method
            // TODO: return the Rotation2d object using the fromDegrees() method
            return null; // TODO remove this line when done.
        };
    }

    public static final Supplier<Rotation2d> createAbsoluteAngleSupplier(
            DutyCycleEncoder dutyCycleEncoder) {
        return () -> {
            // TODO: create a double called angleDegrees and intialize to from the
            // difference of the
            // getAbsolutePosition and getPositionOffset methods from dutyCycleEncoder
            // TODO: return the Rotation2d object using the fromDegrees() method
            return null; // TODO remove this line when done. };
        };
    }
}
