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
    private final AnalogInput encoder;

    public PriorityServoAxonEnc(Servo[] servo, AnalogInput encoder, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, double baseVolt, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers) {
        super(servo, name, type, loadMultiplier, min, max, basePos, reversed, slowdownDist, slowdownPow, basePriority, priorityScale, multipliers);
        this.baseVolt = baseVolt;
        this.encoder = encoder;
    }

    @Override
    public void updateServoValues() {
        double newAngle = ((encoder.getVoltage() + baseVolt) / 3.3) * Math.PI * reversed ;

        reachedIntermediate = Math.abs(currentIntermediateTargetAngle - newAngle) < Math.toRadians(5);

        //updates the current angle the servo thinks it is at
        long currentTime = System.nanoTime();
        double loopTime = ((double) currentTime - lastLoopTime)/1.0E9;
        double error = currentIntermediateTargetAngle - currentAngle;
        double deltaAngle = loopTime * type.speed * (Math.abs(error) <= slowdownDist ? slowdownPow : power) * Math.signum(error);
        //double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);
        reachedIntermediate = Math.abs(deltaAngle) > Math.abs(error);
        if (reachedIntermediate){
            deltaAngle = error;
        }
        currentAngle += deltaAngle;
        lastLoopTime = currentTime;
    }
}
