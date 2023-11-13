package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    public enum State {
        READY,
        BUSY
    }

    private final PriorityMotor slidesMotors;
    public State state = State.READY;
    private double length;
    public double vel;
    private final Sensors sensors;
    public static double ticksToInches = 0.04132142857142857;
    public static double maxSlidesHeight = 28.3465;
    private double targetLength = 0;
    public static double maxVel = 1.6528571428571428;
    public static double kP = 0.07;
    public static double kA = 0.5;
    public static double kStatic = 0.18700000000000014;

    public Slides(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;

        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesMotors = new PriorityMotor(new DcMotorEx[] {m1, m2}, "slidesMotor", 2, 5, new double[] {-1, -1});
        hardwareQueue.addDevice(slidesMotors);
    }

    private double feedforward() {
        double error = targetLength - length;
        TelemetryUtil.packet.put("Error", error);
        return (error * (maxVel / kA)) * kP + kStatic;
    }

    public void update() {
        length = sensors.getSlidesPos() * ticksToInches;
        vel = sensors.getSlidesVelocity() * ticksToInches;
        slidesMotors.setTargetPower(feedforward());
    }

    public void setLength(double length) {
        targetLength = Math.min(length, maxSlidesHeight);
        state = State.BUSY;
    }

    public double getLength() {
        return length;
    }
}
