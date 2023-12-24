package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    public enum State {
        ON,
        OFF,
        REVERSED,
        SOFT_REVERSED
    }

    private final PriorityMotor intake;
    public PriorityServo actuation;
    public State state = State.OFF;
    private final Sensors sensors;
    private final Robot robot;

    public static double intakePower = 1.0; // TODO: Made this editable in FTC dashboard
    private double actuationHeight = 1.0;

    double actuationLength = 3.5;
    double actuationAngle = 0.0;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.sensors = sensors;
        this.robot = robot;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2, -1);
        actuation = new PriorityServo(
                hardwareMap.get(Servo.class,"actuation"),
                "actuation",
                PriorityServo.ServoType.TORQUE,
                1.0,
                0.0,
                1.0,
                0.131,
                false,
                1.0,
                1.0
        );

        this.state = State.OFF;
        hardwareQueue.addDevice(intake);
        hardwareQueue.addDevice(actuation);
    }

    double maxHeightAtParallel = 2.4;

    public void update() {
        actuation.setTargetAngle(actuationAngle, 1.0);

        // TODO: Might need to have a delay bc pixels may not have reached transfer - Huddy kim apparently
        switch (state) {
            case ON:
                if (System.currentTimeMillis() - start > 500) { // don't immediately start the intake until the deposit stalls
                    intake.setTargetPower(intakePower);
                }
                break;
            case OFF:
                intake.setTargetPower(0.0);
                break;
            case REVERSED:
                intake.setTargetPower(-1.0);
                break;
            case SOFT_REVERSED:
                intake.setTargetPower(-0.35);
                break;
        }
    }

    long start;

    public void on() {
        start = System.currentTimeMillis();
        state = State.ON;
    }

    public void off() {
        state = State.OFF;
    }

    public void reverse() {
        state = State.REVERSED;
    }

    public void softReverse() {
        state = State.SOFT_REVERSED;
    }

    public void actuationDown () {
        actuationAngle = 1.378;
    }

    public void actuationUp () {
        actuationAngle = 3.1728;
    }

    public void actuationSinglePixel () {
        actuationAngle = 28;
    }

    public double getIntakeActuationOffset() {
        return Math.cos(actuation.getCurrentAngle()) * actuationLength;
    }
}