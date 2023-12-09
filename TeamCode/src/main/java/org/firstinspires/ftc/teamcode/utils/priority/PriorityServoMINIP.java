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
    public PriorityServoMINIP(Servo servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers, AnalogInput encoder) {
        super(new Servo[]{servo}, name, type, loadMultiplier, min,  max,  basePos,  reversed, slowdownDist, slowdownPow,  basePriority,  priorityScale);
        this.multipliers = multipliers;
        this.encoder = encoder;
        baseAngle = convertAngleToPos(basePos);
    }

    @Override
    public void updateServoValues() {
        double newAngle = AngleUtil.clipAngle(encoder.getVoltage() / 3.3 * 2 * Math.PI + baseAngle);

        currentAngle = newAngle + convertPosToAngle(basePos);
        reachedIntermediate = Math.abs(newAngle - currentAngle) > Math.abs(currentIntermediateTargetAngle - currentAngle);
    }
}
