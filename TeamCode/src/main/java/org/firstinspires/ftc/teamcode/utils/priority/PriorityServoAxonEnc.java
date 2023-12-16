package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;

/**
 * WARNING: Untested class! Use with caution!
 */
public class PriorityServoAxonEnc extends PriorityServo {
    private final double baseVolt;
    private final int encIndex;
    private final Sensors sensors;
    private boolean reversedEnc;

    public PriorityServoAxonEnc(Servo[] servo, Sensors sensors, int encIndex, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, double baseVolt, boolean reversed, boolean reversedEnc, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers) {
        super(servo, name, type, loadMultiplier, min, max, basePos, reversed, slowdownDist, slowdownPow, basePriority, priorityScale, multipliers);
        this.baseVolt = baseVolt;
        this.encIndex = encIndex;
        this.sensors = sensors;
        this.reversedEnc = reversedEnc;
    }

    public PriorityServoAxonEnc(Servo servo, Sensors sensors, int encIndex, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, double baseVolt, boolean reversed, boolean reversedEnc, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers) {
        this(new Servo[] {servo}, sensors, encIndex, name, type, loadMultiplier, min, max, basePos, baseVolt, reversed, reversedEnc, slowdownDist, slowdownPow, basePriority, priorityScale, multipliers);
    }

    public double getEncoderVoltage() {
        return sensors.analogVoltages[encIndex];
    }

    public double getEncoderAngle() {
        return ((getEncoderVoltage() - baseVolt) / 3.3) * 2*Math.PI * (reversedEnc ? -1 : 1);
    }

    @Override
    public void updateServoValues() {
        double newAngle = getEncoderAngle();

        reachedIntermediate = Math.abs(currentIntermediateTargetAngle - newAngle) < Math.toRadians(15);

        currentAngle = newAngle;
    }
}
