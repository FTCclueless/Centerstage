package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * WARNING: Untested class! Use with caution!
 */
public class PriorityServoMINIP extends PriorityServo {
    private final AnalogInput encoder;

    public PriorityServoMINIP(Servo servo, String name, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale, AnalogInput encoder) {
        super(servo, name, PriorityServo.ServoType.AXON_MINI, loadMultiplier, min, max, basePos, reversed, basePriority, priorityScale);
        this.encoder = encoder;
    }

    @Override
    public void updateServoValues() {
        double newAngle = encoder.getVoltage() / 3.3 * 2 * Math.PI;
        reachedIntermediate = Math.abs(newAngle - currentAngle) > Math.abs(currentIntermediateTargetAngle - currentAngle);
        currentAngle = newAngle;
    }
}
