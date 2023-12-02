package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AngleUtil;

/**
 * WARNING: Untested class! Use with caution!
 */
public class PriorityServoMINIP extends PriorityServo {
    private final AnalogInput encoder;
    private final double baseAngle;

    public PriorityServoMINIP(Servo servo, String name, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale, AnalogInput encoder) {
        super(servo, name, PriorityServo.ServoType.AXON_MINI, loadMultiplier, min, max, basePos, reversed, basePriority, priorityScale);
        this.encoder = encoder;
        baseAngle = convertPosToAngle(basePos);
    }

    @Override
    public void updateServoValues() {
        double newAngle = AngleUtil.clipAngle(encoder.getVoltage() / 3.3 * 2 * Math.PI + baseAngle);
        reachedIntermediate = Math.abs(newAngle - currentAngle) > Math.abs(currentIntermediateTargetAngle - currentAngle);
        currentAngle = newAngle;
    }
}
