package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.droppers.Droppers;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

@Config
public class Deposit {
    public enum State {
        START_DEPOSIT,
        FINISH_DEPOSIT,
        DEPOSIT,
        START_RETRACT,
        RETRACT,
        INTAKE,
        GRAB,
        BACK_PICKUP_SETUP,
        BACK_PICKUP,
        BACK_PICKUP_DEPOSIT,
        IDLE,
    };
    public State state;

    public Slides slides;
    public EndAffector endAffector;
    public Release release;
    HardwareQueue hardwareQueue;
    Robot robot;
    Sensors sensors;

    double targetH = 0.0;
    double targetX = 0.0;

    public static double slidesV4Thresh = 2.5;

    // v4bar angles
    public static double v4BarTransferAngle = -0.29137;
    public static double v4BarGrabAngle = -0.0392242;
    public static double v4BarDepositAngle = -2.97538;

    // top servo angles
    public static double topServoTransferAngle = -0.99181222;
    public static double topServoGrabAngle = -0.94698;
    public static double topServoDepositAngle = 2.101297;
    public static double topServoRetractAngle = 2.6336256;

    // back pickup
    public static double v4BarBackPickupAngle = 0.0;
    public static double topServoBackPickupAngle = 0.0;
    public static double slidesBackPickupHeight = 0.0;
    public static double slidesBackPickupHeightDown = 0.0;

    // back pickup deposit
    public static double v4BarBackPickupDepositAngle = 0.0;
    public static double topServoBackPickupDepositAngle = 0.0;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.robot = robot;

        state = State.GRAB;

        slides = new Slides(hardwareMap, hardwareQueue, sensors);
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
        release = new Release(hardwareMap, hardwareQueue);
    }

    public void depositAt(Vector3 vector3) {
        depositAt(vector3.z, vector3.x);
    }

    boolean startDeposit = false;
    // Call this whenever you want! It can be an updating function!
    public void depositAt(double targetH, double targetX) {
        this.targetH = Math.max(Math.min(targetH, Slides.maxSlidesHeight),0);
        this.targetX = targetX;

        if (state == State.INTAKE) {
            state = State.GRAB;
            beginGrabTime = System.currentTimeMillis();
            startDeposit = true;
        }
        else if (state == State.GRAB) {
            startDeposit = true;
        }
    }

    public void releaseOne() {
        release.releaseOne();
    }

    public void resetPixelCount() {
        Globals.NUM_PIXELS = 2;
    }

    public void releaseTwo() {
        release.releaseTwo();
    }

    public void retract() {
        beginRetractTime = System.currentTimeMillis();
        state = State.START_RETRACT;
    }

    public void backPickupSetup() {
        beginBackPickupSetupTime = System.currentTimeMillis();
        state = State.BACK_PICKUP_SETUP;
    }

    public void backPickup() {
        beginBackPickupTime = System.currentTimeMillis();
        state = State.BACK_PICKUP;
    }

    public boolean checkReady() {
        return endAffector.checkReady() && !slides.inPosition(2);
    }

    long beginDepositTime;
    long beginGrabTime = System.currentTimeMillis();
    long beginRetractTime;
    long beginBackPickupSetupTime;
    long beginBackPickupTime;

    public void update() {
        TelemetryUtil.packet.put("Deposit State", state);

        switch (state) {
            case INTAKE:
                release.intake();
                slides.setTargetLength(0.0);

                endAffector.v4Servo.setTargetAngle(v4BarTransferAngle,0.75);
                endAffector.topServo.setTargetAngle(topServoTransferAngle,1.0);

                if (robot.intake.state != Intake.State.ON) { // check if intake is off
                    beginGrabTime = System.currentTimeMillis();
                    state = State.GRAB;
                }
                break;
            case GRAB:
                endAffector.v4Servo.setTargetAngle(v4BarGrabAngle, 0.75);
                endAffector.topServo.setTargetAngle(topServoGrabAngle, 1.0);
                release.preGrab();

                if (robot.intake.state == Intake.State.ON && !startDeposit) {
                    state = State.INTAKE;
                }

                if (startDeposit && System.currentTimeMillis() - beginGrabTime > 250) {
                    beginDepositTime = System.currentTimeMillis();
                    state = State.START_DEPOSIT;
                }
                break;
            case START_DEPOSIT:
                release.close();
                if (System.currentTimeMillis() - beginDepositTime > 150) {
                    slides.setTargetLength(Math.max(targetH, slidesV4Thresh + 2));
                }

                if (slides.getLength() > slidesV4Thresh) {
                    state = State.FINISH_DEPOSIT;
                }
                break;
            case FINISH_DEPOSIT: // stuck in this state unless someone calls dunk method. In the meantime it will constantly update targetH
                startDeposit = false;
                slides.setTargetLength(targetH);
                endAffector.v4Servo.setTargetAngle(v4BarDepositAngle,0.75);
                endAffector.topServo.setTargetAngle(topServoDepositAngle,1.0);

                if (endAffector.v4Servo.inPosition() && slides.inPosition(2)) {
                    state = State.DEPOSIT;
                }
                break;
            case DEPOSIT:
                slides.setTargetLength(targetH);

                if (release.readyToRetract()) {
                    beginRetractTime = System.currentTimeMillis();
                    state = State.START_RETRACT;
                }
                break;
            case START_RETRACT:
                Log.e("System.currentTimeMillis()", System.currentTimeMillis() + "");
                Log.e("beginRetractTime", beginRetractTime + "");
                endAffector.topServo.setTargetAngle(topServoRetractAngle, 1.0);
                if (System.currentTimeMillis() - beginRetractTime > 200) {
                    release.close();
                    endAffector.v4Servo.setTargetAngle(v4BarTransferAngle, 0.75);

                    if (endAffector.v4Servo.getCurrentAngle() <= Math.toRadians(135)) {
                        slides.setTargetLength(slidesV4Thresh + 2);
                        if (endAffector.v4Servo.getCurrentAngle() <= Math.toRadians(90)) {
                            endAffector.topServo.setTargetAngle(topServoTransferAngle, 1.0);
                        } else {
                            endAffector.topServo.setTargetAngle(0, 1.0);
                        }
                    } else {
                        slides.setTargetLength(targetH + 3); // TODO: Fix this to make the arm wait for slides to go up then move
                    }

                    if (endAffector.v4Servo.inPosition()) {
                        state = State.RETRACT;
                    }
                }
                break;
            case RETRACT:
                slides.setTargetLength(0.0);

                if (slides.inPosition(2)) {
                    state = State.INTAKE;
                }
                break;
            case BACK_PICKUP_SETUP:
                slides.setTargetLength(Math.max(slidesBackPickupHeight, slidesV4Thresh + 2));
                if (slides.getLength() > slidesV4Thresh) {
                    endAffector.v4Servo.setTargetAngle(v4BarBackPickupAngle, 1.0);
                    endAffector.topServo.setTargetAngle(topServoBackPickupAngle, 1.0);
                    if (System.currentTimeMillis() - beginBackPickupSetupTime > 300) {
                        slides.setTargetLength(slidesBackPickupHeight);
                    }
                } else {
                    beginBackPickupSetupTime = System.currentTimeMillis();
                }
                break;
            case BACK_PICKUP:
                slides.setTargetLength(slidesBackPickupHeightDown);
                Globals.NUM_PIXELS = 1;
                if (slides.inPosition(0.5)) {
                    release.close();
                    if (System.currentTimeMillis() - beginBackPickupSetupTime > 500) {
                        state = State.BACK_PICKUP_DEPOSIT;
                    }
                } else {
                    beginBackPickupTime = System.currentTimeMillis();
                }
                break;
            case BACK_PICKUP_DEPOSIT:
                slides.setTargetLength(targetH);
                endAffector.v4Servo.setTargetAngle(v4BarBackPickupDepositAngle, 1.0);
                endAffector.topServo.setTargetAngle(topServoBackPickupDepositAngle, 1.0);

                if (release.readyToRetract()) {
                    beginRetractTime = System.currentTimeMillis();
                    state = State.START_RETRACT;
                }
                break;
            case IDLE: // We are boring :(
                break;
        }
        slides.update();
        release.update();

        TelemetryUtil.packet.put("v4ServoAngle", endAffector.v4Servo.getCurrentAngle());
        TelemetryUtil.packet.put("v4ServoTarget", endAffector.v4Servo.getTargetPosition());
    }
}