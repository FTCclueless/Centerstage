package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    public enum MotorState {
        ON,
        OFF,
        REVERSED,
        SOFT_REVERSED,
        REVERSE_FOR_TIME
    }

    public final PriorityMotor intake;
    public PriorityServo actuation;
    public MotorState motorState = MotorState.OFF;
    private final Sensors sensors;
    private final Robot robot;
    public double intakeCurrent = 0;
    public double dist = 0;

    public static double intakePower = 1.0; // TODO: Made this editable in FTC dashboard

    double actuationLength = 3.5;
    double[] actuationAngles = new double[] {0.62901, 0.47176, 0.365384, 0.231256, 0.14568993}; // 1 pixel --> 5 pixle
    double actuationFullyUpAngle = -1.417675;

    // stall checking variables
    double intakeDebounce;
    double stallStart;
    double intakeCheck;
    public static double stallThresh = 4500;
    public final REVColorSensorV3 colorSensorV3;

    enum StallState {
        CHECK,
        CONFIRM,
        UNSTALL
    }

    StallState stallState = StallState.CHECK;

    enum PixelCheckState {
        CHECK,
        CONFIRM,
        GO_REVERSE
    }
    private PixelCheckState pixelCheckState = PixelCheckState.CHECK;
    private long lastProxPoll = System.currentTimeMillis();
    public static int pixelTouchingDist = 270;
    private double confirmtionLoops = 0;
    public static double desiredConfirmtionLoops = 10;
    private long goReverseStart = 0;
    public static double goReverseDelay = 350;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.sensors = sensors;
        this.robot = robot;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2, -1, sensors);
        actuation = new PriorityServo(
                hardwareMap.get(Servo.class,"actuation"),
                "actuation",
                PriorityServo.ServoType.AXON_MINI,
                1.0,
                0.0,
                1.0,
                0.402,
                false,
                1.0,
                1.0
        );
        colorSensorV3 = hardwareMap.get(REVColorSensorV3.class, "intakeColorSensor");
        REVColorSensorV3.ControlRequest req = new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.PROX_SENSOR_ENABLED);
        colorSensorV3.sendControlRequest(req);
        colorSensorV3.configurePS(REVColorSensorV3.PSResolution.EIGHT, REVColorSensorV3.PSMeasureRate.m6p25s);

        this.motorState = MotorState.OFF;
        hardwareQueue.addDevice(intake);
        hardwareQueue.addDevice(actuation);

        intakeCheck = System.currentTimeMillis();
    }

    public void update() {
        TelemetryUtil.packet.put("Intake Motor State", motorState);
        TelemetryUtil.packet.put("Intake Stall State", stallState);
        TelemetryUtil.packet.put("Intake Pixel Check State", pixelCheckState);

        TelemetryUtil.packet.put("Intake Current", intakeCurrent);
        TelemetryUtil.packet.put("Intake Pixel Color Sensor Dist", dist);

        switch (motorState) {
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
                    motorState = previousState;
                }
                break;
        }

        switch (stallState) {
            case CHECK:
                if (motorState != MotorState.ON) {
                    intakeCheck = System.currentTimeMillis();
                }

                if (System.currentTimeMillis() - intakeCheck > 150) {
                    intakeCurrent = intake.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
                    intakeCheck = System.currentTimeMillis();
                    intakeDebounce = System.currentTimeMillis();
                    if (intakeCurrent > stallThresh) {
                        stallStart = System.currentTimeMillis();
                        stallState = StallState.CONFIRM;
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
                    stallState = StallState.CHECK;
                }
                if (intakeDebounce - stallStart > 250) {
                    intakeCheck = System.currentTimeMillis();
                    stallState = StallState.UNSTALL;
                }
                break;
            case UNSTALL:
                Log.e("JAM REVERSE FALLBACK", "-----");
                reverseForSomeTime(750);
                stallState = StallState.CHECK;
                break;
        }

        switch (pixelCheckState) {
            case CHECK:
                if (motorState == MotorState.ON && System.currentTimeMillis() - lastProxPoll > 100) {
                    dist = colorSensorV3.readPS();
                    if (dist >= pixelTouchingDist)
                        pixelCheckState = PixelCheckState.CONFIRM;
                    lastProxPoll = System.currentTimeMillis();
                }
                break;
            case CONFIRM:
                // Super polling
                dist = colorSensorV3.readPS();

                if (dist < pixelTouchingDist)
                    pixelCheckState = PixelCheckState.CHECK;

                if (confirmtionLoops++ >= desiredConfirmtionLoops) {
                    confirmtionLoops = 0;
                    pixelCheckState = PixelCheckState.GO_REVERSE;
                    goReverseStart = System.currentTimeMillis();
                }
                break;
            case GO_REVERSE:
                Log.e("PIXEL REVERSE FALLBACK", "-----");
                Globals.NUM_PIXELS = 2;
                // Wait a bit then reverse for some time
                if (System.currentTimeMillis() - goReverseStart > goReverseDelay) {
                    // Jankly set the previous state so reverseForSomeTime will turn the intake off
                    motorState = MotorState.OFF;
                    reverseForSomeTime(750);
                    pixelCheckState = PixelCheckState.CHECK;
                }

                break;
        }
    }

    public void on() {
        motorState = MotorState.ON;
    }

    public void off() {
        motorState = MotorState.OFF;
    }

    public void reverse() {
        motorState = MotorState.REVERSED;
    }

    long reverseForSomeTimeStart;
    double time;
    MotorState previousState;
    public void reverseForSomeTime(double time) {
        this.time = time;
        reverseForSomeTimeStart = System.currentTimeMillis();

        if (motorState != MotorState.REVERSE_FOR_TIME) {
            previousState = motorState;
        }
        motorState = MotorState.REVERSE_FOR_TIME;
    }

    public void softReverse() {
        motorState = MotorState.SOFT_REVERSED;
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