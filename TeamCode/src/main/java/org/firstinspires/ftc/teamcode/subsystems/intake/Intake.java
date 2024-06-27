package org.firstinspires.ftc.teamcode.subsystems.intake;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Func;
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

    public static double intakePower = 1.0;

    double actuationLength = 3.5;

//    double[] actuationAngles = new double[] {1.40086, 0.8069808, 0.42338047, 0.388276711, 0.15519023 /* 0.04 g */}; // 1 pixel --> 5 pixel
//    double actuationFullyUpAngle = -1.9724175;
    double[] actuationAngles = new double[] {0.885346505, 0.448276711, 0.364224828, 0.1625003, 0.005603458}; // 1 pixel --> 5 pixel
    double actuationFullyUpAngle = -2.196550284;

    // stall checking variables
    double intakeDebounce;
    double stallStart;
    double intakeCheck;
    public static double reversedPower = -1.0;
    public static double timeToReverse = 100;
    public static double stallThresh = 6000;
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
    public static int pixelTouchingDist = 315;
    private double confirmationLoops = 0;
    public static double desiredConfirmationLoops = 30;
    private long goReverseStart = 0;
    public static double goReverseDelay = 370;
    public long reversedTime = -1; // Used to tell other subsystems when we have last reversed the intake b.c. jam or 2 many pixels (-1 if not happened)

    public boolean useIntakeStallCheck = true;
    public boolean useIntakeColorSensorCheck = true;
    public boolean reversed = false;

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
                0.391999,
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
                long elapsed = System.currentTimeMillis() - reverseForSomeTimeStart;
                if (elapsed < time) {
                    intake.setTargetPower(reversedPower * Math.sin((Math.PI / (time * 2)) * elapsed)); // Pulse once
                } else {
                    motorState = previousState;
                }
                break;
        }

        if  (useIntakeStallCheck) {
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
                        Log.e("JAM REVERSE FALLBACK", "-----");
                        stallState = StallState.UNSTALL;
                    }
                    break;
                case UNSTALL:
                    reversedTime = System.currentTimeMillis();
                    reverseForSomeTime(timeToReverse);
                    stallState = StallState.CHECK;
                    break;
            }
        }

        if (useIntakeColorSensorCheck) {
            switch (pixelCheckState) {
                case CHECK:
                    confirmationLoops = 0;

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

                    if (confirmationLoops++ >= desiredConfirmationLoops) {
                        confirmationLoops = 0;
                        pixelCheckState = PixelCheckState.GO_REVERSE;
                        goReverseStart = System.currentTimeMillis();
                        Log.e("2 PIXEL REVERSE FALLBACK", "-----");
                    }
                    break;
                case GO_REVERSE:
                    reversed = true;
                    reversedTime = System.currentTimeMillis();
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
        if (pixelIndex < 0) {
            pixelIndex = 0;
        }
        if (pixelIndex > 4) {
            pixelIndex = 4;
        }
        actuation.setTargetAngle(actuationAngles[pixelIndex], 1.0);
    }

    public void setActuationHeightSlightlyAbove (int pixelIndex) {
        if (pixelIndex < 0) {
            pixelIndex = 0;
        }
        if (pixelIndex > 4) {
            pixelIndex = 4;
        }
        actuation.setTargetAngle(actuationAngles[pixelIndex] - 0.2, 1.0);
        robot.update();
    }

    public void setActuationHeight (int pixelIndex, double power) {
        if (pixelIndex < 0) {
            pixelIndex = 0;
        }
        if (pixelIndex > 4) {
            pixelIndex = 4;
        }
        actuation.setTargetAngle(actuationAngles[pixelIndex], power);
    }

    public void setActuationAngle(double angle, double power) { // 0 index based
        actuation.setTargetAngle(angle, power);
    }

    public boolean actuationReady() {
        return actuation.inPosition();
    }

    public boolean twoPixelsInTransfer() {
        return colorSensorV3.readPS() >= pixelTouchingDist;
    }

    public double getIntakeActuationOffset() {
        return Math.cos(actuation.getCurrentAngle()) * actuationLength;
    }

    public boolean isActuationUp() {
        return actuation.getCurrentAngle() == actuationFullyUpAngle;
    }

    public int forcePullColorSensorDist() {
        return colorSensorV3.readPS();
    }
}