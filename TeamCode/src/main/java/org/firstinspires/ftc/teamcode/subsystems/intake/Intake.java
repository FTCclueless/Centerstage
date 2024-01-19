package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    public enum IntakeMotorState {
        ON,
        OFF,
        REVERSED,
        SOFT_REVERSED,
        REVERSE_FOR_TIME
    }

    public final PriorityMotor intake;
    public PriorityServo actuation;
    public IntakeMotorState intakeMotorState = IntakeMotorState.OFF;
    private final Sensors sensors;
    private final Robot robot;
    public static double intakeCurrent;

    public static double intakePower = 1.0; // TODO: Made this editable in FTC dashboard

    double actuationLength = 3.5;
    double[] actuationAngles = new double[] {0.62901, 0.47176, 0.365384, 0.231256, 0.13412};
    double actuationFullyUpAngle = -0.314508;

    // stall checking variables
    double intakeDebounce;
    double stallStart;
    double intakeCheck;
    public static double stallThresh = 4500;

    enum IntakeStallState {
        CHECK,
        CONFIRM,
        UNSTALL
    }
    IntakeStallState intakeStallState = IntakeStallState.CHECK;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.sensors = sensors;
        this.robot = robot;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2, -1, sensors);
        actuation = new PriorityServo(
                hardwareMap.get(Servo.class,"actuation"),
                "actuation",
                PriorityServo.ServoType.TORQUE,
                1.0,
                0.0,
                1.0,
                0.068,
                false,
                1.0,
                1.0
        );

        this.intakeMotorState = IntakeMotorState.OFF;
        hardwareQueue.addDevice(intake);
        hardwareQueue.addDevice(actuation);

        intakeCheck = System.currentTimeMillis();
    }

    public void update() {
        TelemetryUtil.packet.put("Intake Motor State", intakeMotorState);
        TelemetryUtil.packet.put("Intake Stall State", intakeStallState);

        switch (intakeMotorState) {
            case ON:
                intake.setTargetPower(intakePower);
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
            case REVERSE_FOR_TIME:
                if (System.currentTimeMillis() - reverseForSomeTimeStart < time) {
                    intake.setTargetPower(-1.0);
                } else {
                    intakeMotorState = previousState;
                }
                break;
        }

        switch (intakeStallState) {
            case CHECK:
                if (intakeMotorState != IntakeMotorState.ON) {
                    intakeCheck = System.currentTimeMillis();
                }

                if (System.currentTimeMillis() - intakeCheck > 150) {
                    intakeCurrent = intake.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
                    intakeCheck = System.currentTimeMillis();
                    intakeDebounce = System.currentTimeMillis();
                    if (intakeCurrent > stallThresh) {
                        stallStart = System.currentTimeMillis();
                        intakeStallState = IntakeStallState.CONFIRM;
                    }
                }
                break;
            case CONFIRM:
                intakeCurrent = intake.motor[0].getCurrent(CurrentUnit.MILLIAMPS);

                if (intakeCurrent > stallThresh) {
                    intakeDebounce = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - intakeDebounce > 100) {
                    intakeCheck = System.currentTimeMillis();
                    intakeStallState = IntakeStallState.CHECK;
                }
                if (intakeDebounce - stallStart > 250) {
                    intakeCheck = System.currentTimeMillis();
                    intakeStallState = IntakeStallState.UNSTALL;
                }
                break;
            case UNSTALL:
                reverseForSomeTime(750);
                intakeStallState = IntakeStallState.CHECK;
                break;
        }
    }

    public void on() {
        intakeMotorState = IntakeMotorState.ON;
    }

    public void off() {
        intakeMotorState = IntakeMotorState.OFF;
    }

    public void reverse() {
        intakeMotorState = IntakeMotorState.REVERSED;
    }

    long reverseForSomeTimeStart;
    double time;
    IntakeMotorState previousState;
    public void reverseForSomeTime(double time) {
        this.time = time;
        reverseForSomeTimeStart = System.currentTimeMillis();

        if (intakeMotorState != IntakeMotorState.REVERSE_FOR_TIME) {
            previousState = intakeMotorState;
        }
        intakeMotorState = IntakeMotorState.REVERSE_FOR_TIME;
    }

    public void softReverse() {
        intakeMotorState = IntakeMotorState.SOFT_REVERSED;
    }

    public void actuationFullyDown() {
        actuation.setTargetAngle(actuationAngles[0],1.0);
    }

    public void actuationFullyUp() {
        actuation.setTargetAngle(actuationFullyUpAngle, 1.0);
    }

    public void setActuationHeight (int pixelIndex) {
        actuation.setTargetAngle(actuationAngles[pixelIndex], 1.0);
    }

    public void setActuationAngle(double angle, double power) { // 0 index based
        actuation.setTargetAngle(angle, power);
    }

    public double getIntakeActuationOffset() {
        return Math.cos(actuation.getCurrentAngle()) * actuationLength;
    }

    public boolean isActuationUp() {
        return actuation.getCurrentAngle() == actuationFullyUpAngle;
    }
}