package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Utils;

public class PriorityMotor extends PriorityDevice{
    double lastPower = 0;
    public double power = 0;
    public DcMotorEx[] motor; // if the subsystem has multiple motors (i.e. slides)
    public String name;

    private double minPowerToOvercomeFriction = 0.0;

    public PriorityMotor(DcMotorEx motor, String name, double basePriority, double priorityScale) {
        this(new DcMotorEx[] {motor}, name, basePriority, priorityScale);
    }

    public PriorityMotor(DcMotorEx[] motor, String name, double basePriority, double priorityScale) {
        super(basePriority, priorityScale, name);
        this.motor = motor;
        this.name = name;

        minPowerToOvercomeFriction = 0.0;
        callLengthMillis = 1.6;
    }

    public void setMinimumPowerToOvercomeFriction (double value) {
        minPowerToOvercomeFriction = value;
    }
    public void setTargetPower(double power) {
        power = Utils.minMaxClip(power, -1.0, 1.0);
        this.power = power + (minPowerToOvercomeFriction * Math.signum(power));
    }

    public double getPower() {
        return power;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if (power-lastPower == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        if (timeRemaining * 1000.0 <= callLengthMillis * (motor.length-1) + callLengthMillis/2.0) {
            return 0;
        }

        return basePriority + Math.abs(power-lastPower) + (System.nanoTime() - lastUpdateTime)/1000.0 * priorityScale;
    }

    @Override
    protected void update() {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power);
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
}
