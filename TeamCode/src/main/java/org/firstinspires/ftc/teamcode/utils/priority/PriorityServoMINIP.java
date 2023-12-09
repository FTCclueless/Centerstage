package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;

/**
 * WARNING: Untested class! Use with caution!
 */
public class PriorityServoMINIP extends PriorityServo {
    private final double baseAngle;
    Sensors sensors;
    int analogInput;

    public PriorityServoMINIP(Servo servo, String name, Sensors sensors, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale, int analogInput) {
        super(servo, name, PriorityServo.ServoType.AXON_MINI, loadMultiplier, min, max, basePos, reversed, basePriority, priorityScale);
        baseAngle = convertPosToAngle(basePos);
        this.analogInput = analogInput;
    }
    public PriorityServoMINIP(Servo servo, String name, ServoType type, Sensors sensors, double loadMultiplier, double min, double max, double basePos, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers, int analogInput) {
        super(new Servo[]{servo}, name, type, loadMultiplier, min,  max,  basePos,  reversed, slowdownDist, slowdownPow,  basePriority,  priorityScale);
        this.multipliers = multipliers;
        baseAngle = convertAngleToPos(basePos);
        this.analogInput = analogInput;
    }

    @Override
    public void updateServoValues() {
        double encoderVolt = 0;
        switch (analogInput) {
            case 0:
                encoderVolt = sensors.getAnalog0Volt();
                break;
            case 1:
                encoderVolt = sensors.getAnalog1Volt();
                break;
            case 2:
                encoderVolt = sensors.getAnalog2Volt();
                break;
            case 3:
                encoderVolt = sensors.getAnalog3Volt();
                break;
        }
        double newAngle = encoderVolt / 3.3 * 2 * Math.PI + baseAngle;

        currentAngle = newAngle + convertPosToAngle(basePos);
        reachedIntermediate = Math.abs(newAngle - currentAngle) < Math.toRadians(5);
    }
}
