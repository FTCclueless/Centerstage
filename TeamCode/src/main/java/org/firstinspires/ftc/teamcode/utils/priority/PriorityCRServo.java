package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.CRServo;

public class PriorityCRServo extends PriorityDevice {
    public CRServo[] servo;
    double power = 0;
    double lastPower = 0;
    public PriorityCRServo(CRServo servo, String name,  double basePriority, double priorityScale) {
        super(basePriority, priorityScale, name);
        this.servo = new CRServo[]{servo};
    }
    public PriorityCRServo(CRServo[] servos, String name, double basePriority, double priorityScale) { //one of the servos must be reversed prior to use
        super(basePriority, priorityScale, name);
        this.servo = servos;
    }

    public void setTargetPower(double power) {
        this.power = power;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if (power-lastPower == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }
        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }
        return basePriority + Math.abs(power-lastPower) + (System.nanoTime()-lastUpdateTime)/1000000.0  * priorityScale;
    }

    @Override
    protected void update() {
        for (CRServo servo : servo) {
            servo.setPower(power);
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
}
