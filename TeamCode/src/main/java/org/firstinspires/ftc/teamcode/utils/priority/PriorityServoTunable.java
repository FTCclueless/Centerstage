package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.Servo;

public class PriorityServoTunable extends PriorityServo {
    double positionPerRadian;
    public PriorityServoTunable(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, double minAng, double maxAng, boolean reversed, double basePriority, double priorityScale, double[] multipliers) {
        super(servo, name, type, loadMultiplier, min, max, basePos, reversed, basePriority, priorityScale, multipliers);
        this.minAng = minAng;
        this.maxAng = maxAng;

        positionPerRadian = (maxPos-minPos)/(maxAng-minAng);

        if (reversed) {
            positionPerRadian *= -1;
        }
    }
    public PriorityServoTunable(Servo servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, double minAng, double maxAng, boolean reversed, double basePriority, double priorityScale) {
        super(servo,name,type,loadMultiplier,min,max,basePos,reversed,basePriority,priorityScale);
        this.minAng = minAng;
        this.maxAng = maxAng;

        positionPerRadian = (maxPos-minPos)/(maxAng-minAng);

        if (reversed) {
            positionPerRadian *= -1;
        }
    }

    @Override
    public double convertAngleToPos(double ang) {
        return Math.max(Math.min((ang-minAng) * positionPerRadian, maxAng),minAng)+minPos;
    }


}
