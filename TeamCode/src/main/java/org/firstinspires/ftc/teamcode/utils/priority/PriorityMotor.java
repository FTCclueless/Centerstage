package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class PriorityMotor extends PriorityDevice {
    double lastPower = 0;
    public double power = 0;
    public DcMotorEx[] motor; // if the subsystem has multiple motors (i.e. slides)
    private double[] multipier;
    public String name;

    Sensors sensors;

    private double minPowerToOvercomeStaticFriction = 0.0;
    public static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 75;
    private double minPowerToOvercomeKineticFriction = 0.0;
    private long lastZeroTime = 0;

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

        minPowerToOvercomeStaticFriction = 0.0;
        minPowerToOvercomeKineticFriction = 0.0;
        lastZeroTime = System.currentTimeMillis();
        callLengthMillis = 1.6;
        this.multipier = multiplier;
    }

    public void setMinimumPowerToOvercomeStaticFriction (double value) {
        minPowerToOvercomeStaticFriction = value;
    }

    public void setMinimumPowerToOvercomeKineticFriction(double value) {
        minPowerToOvercomeKineticFriction = value;
    }

    public void setTargetPower(double power) {
        if (lastPower == 0) {
            lastZeroTime = System.currentTimeMillis();
        }
        power = Utils.minMaxClip(power, -1.0, 1.0);
        double m = (System.currentTimeMillis() > SWITCH_FROM_STATIC_TO_KINETIC_FRICTION + lastZeroTime ? minPowerToOvercomeKineticFriction : minPowerToOvercomeStaticFriction) * (12/sensors.getVoltage());
        power *= 1-m;
        this.power = power + m * Math.signum(power);
    }

    double k = 0.7; // 0.5
    public void setTargetPowerSmooth(double power) {
        if (lastPower == 0){
            lastZeroTime = System.currentTimeMillis();
        }
        if (power == 0) {
            this.power = 0;
            return;
        }
        power = Utils.minMaxClip(power, -1.0, 1.0);
        double m = (System.currentTimeMillis() > SWITCH_FROM_STATIC_TO_KINETIC_FRICTION + lastZeroTime ? minPowerToOvercomeKineticFriction : minPowerToOvercomeStaticFriction) * (13.5/sensors.getVoltage());
        power *= 1-m;
        power = power + m * Math.signum(power);
        this.power = power*k + this.lastPower*(1-k);
    }

    public void setPowerForced(double power) {
        if (power == 0)
            lastZeroTime = System.currentTimeMillis();

        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power * multipier[i]);
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
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

        return basePriority + priorityScale * (System.nanoTime() - lastUpdateTime)/1.0E6 * Math.abs(power-lastPower);
    }

    @Override
    protected void update() {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power * multipier[i]);
        }
        if (power == 0){
            lastZeroTime = System.currentTimeMillis();
        }
        lastUpdateTime = System.nanoTime();
        lastPower = power;
    }
}
