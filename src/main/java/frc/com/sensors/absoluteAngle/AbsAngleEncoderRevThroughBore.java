package frc.com.sensors.absoluteAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsAngleEncoderRevThroughBore implements AbsAngleEncoder {

    private static final class Constants{
        private static final double periodMicroSeconds = 1025.0;
        private static final double dutyCycleMin = 1.0 / periodMicroSeconds;
        private static final double dutyCycleMax = (periodMicroSeconds - 1) / periodMicroSeconds;
    }

    private final DutyCycleEncoder dutyCycleEncoder;

    public AbsAngleEncoderRevThroughBore(int dioChannelNumber, double offset){
        dutyCycleEncoder = new DutyCycleEncoder(dioChannelNumber);
        dutyCycleEncoder.setPositionOffset(offset);
        dutyCycleEncoder.setDutyCycleRange(Constants.dutyCycleMin, Constants.dutyCycleMax);
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        double angleDegrees = dutyCycleEncoder.getAbsolutePosition() - dutyCycleEncoder.getPositionOffset();
        return Rotation2d.fromDegrees(angleDegrees);
    }

    @Override
    public boolean isDataGood() {
        return dutyCycleEncoder.isConnected();
    }

}
