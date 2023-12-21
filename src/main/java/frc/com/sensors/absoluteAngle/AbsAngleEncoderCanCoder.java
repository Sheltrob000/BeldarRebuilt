package frc.com.sensors.absoluteAngle;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import edu.wpi.first.math.geometry.Rotation2d;

public class AbsAngleEncoderCanCoder implements AbsAngleEncoder {

    private final CANCoder cancoder;

    public AbsAngleEncoderCanCoder(int deviceNumber, String canbus) {
        cancoder = new CANCoder(deviceNumber, canbus);
        cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        double angleDegrees = cancoder.getAbsolutePosition();
        return Rotation2d.fromDegrees(angleDegrees);
    }

    @Override
    public boolean isDataGood() {
        MagnetFieldStrength magnetFieldStrength = cancoder.getMagnetFieldStrength();
        return magnetFieldStrength == MagnetFieldStrength.Good_GreenLED;
    }

}