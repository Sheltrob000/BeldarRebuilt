package frc.com.sensors.absoluteAngle;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AbsAngleEncoder {

    public Rotation2d getAbsoluteAngle();

    public boolean isDataGood();
}
