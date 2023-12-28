package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class PriorityMotor extends PriorityDevice{
    double lastPower = 0;
    public double power = 0;
    public DcMotorEx[] motor; // if the subsystem has multiple motors (i.e. slides)
    private double[] multipier;
    public String name;

    Sensors sensors;

    private double minPowerToOvercomeFriction = 0.0;

    public PriorityMotor(DcMotorEx motor, String name, double basePriority, double priorityScale, double multiplier, Sensors sensors) {
        this(new DcMotorEx[] {motor}, name, basePriority, priorityScale, new double[]{multiplier}, sensors);
    }

    public PriorityMotor(DcMotorEx motor, String name, double basePriority, double priorityScale, Sensors sensors) {
        this(new DcMotorEx[] {motor}, name, basePriority, priorityScale, new double[]{1}, sensors);
    }

    public PriorityMotor(DcMotorEx[] motor, String name, double basePriority, double priorityScale, double[] multiplier, Sensors sensors) {
        super(basePriority, priorityScale, name);
        this.motor = motor;
        this.name = name;
        this.sensors = sensors;

        minPowerToOvercomeFriction = 0.0;
        callLengthMillis = 1.6;
        this.multipier = multiplier;
    }

    public void setMinimumPowerToOvercomeFriction (double value) {
        minPowerToOvercomeFriction = value;
    }

    public void setTargetPower(double power) {
        power = Utils.minMaxClip(power, -1.0, 1.0);
        power *= 1-minPowerToOvercomeFriction;
        this.power = power + (minPowerToOvercomeFriction * (12/sensors.getVoltage()) * Math.signum(power));
        if (power == 0) {
            this.power = 0;
        }
    }

    public double getPower() {
        return power;
    }

    public double getVelocity() {
        return motor[0].getVelocity();
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

        return basePriority + Math.abs(power-lastPower) + (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;
    }

    @Override
    protected void update() {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power * multipier[i]);
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
}
