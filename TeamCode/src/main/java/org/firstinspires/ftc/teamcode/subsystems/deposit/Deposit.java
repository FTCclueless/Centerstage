package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
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

    // v4bar angles
    public static double v4BarTransferAngle =  -0.718570101944; //-0.3362075338;
    public static double v4BarGrabAngle = -0.415345273;
    public static double v4BarDepositAngle = -3.5556096189;

    // top servo angles
    public static double topServoTransferAngle = -0.9525880; //-0.94698455;
    public static double topServoGrabAngle = -0.975001848;
    public static double topServoDepositAngle = 2.15727979;
    public static double topServoRetractAngle = 0.7060358;

    // pixel readjustment mode
    public static double v4BarReadjustAngle = -4.08880047; // -3.9896627
    public static double topServoReadjustAngle = -0.70603582; // -1.09827794

    public static double autoTopServoDepositAngle = 2.07327979;

    public static double v4BarInitAngle =  -0.56594934;
    public static double topServoInitAngle =  -1.047846813;

    public boolean isSlowSlidesMode = false;

    // old readjustment values
//    public static double v4BarReadjustAngle = -2.75124228;
//    public static double topServoReadjustAngle = 0.5435355;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.robot = robot;

        state = State.GRAB;

        slides = new Slides(hardwareMap, hardwareQueue, sensors, robot.drivetrain, this);
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
        release = new Release(hardwareMap, hardwareQueue);

        if (Globals.RUNMODE == RunMode.AUTO) {
            topServoDepositAngle = autoTopServoDepositAngle;
        }
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

    public boolean isDepositing() {
        return state == State.DEPOSIT || state == State.START_DEPOSIT || state == State.FINISH_DEPOSIT;
    }

    public void retract() {
        beginRetractTime = System.currentTimeMillis();
        state = State.START_RETRACT;
    }

    public void retractInit() {
        release.close();
        endAffector.v4Servo.setTargetAngle(v4BarInitAngle, 1.0);
        endAffector.topServo.setTargetAngle(topServoTransferAngle, 1.0);
    }

    public boolean checkReady() {
        return endAffector.checkReady() && slides.inPosition(2);
    }

    long beginDepositTime;
    long beginGrabTime = System.currentTimeMillis();
    long beginRetractTime;

    public static double v4ServoPower = 0.85;
    public static double topServoPower = 1.0;

    public void update() {
        TelemetryUtil.packet.put("Deposit State", state);

        switch (state) {
            case INTAKE:
                release.intake();
                slides.setTargetLength(0.0);

                endAffector.v4Servo.setTargetAngle(v4BarTransferAngle,v4ServoPower);
                endAffector.topServo.setTargetAngle(topServoTransferAngle,topServoPower);

                if (robot.intake.motorState != Intake.MotorState.ON) { // check if intake is off
                    beginGrabTime = System.currentTimeMillis();
                    state = State.GRAB;
                }
                break;
            case GRAB:
                endAffector.v4Servo.setTargetAngle(v4BarGrabAngle, v4ServoPower);
                endAffector.topServo.setTargetAngle(topServoGrabAngle, topServoPower);
                release.preGrab();

                if (robot.intake.motorState == Intake.MotorState.ON && !startDeposit) {
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
                    slides.setTargetLength(Math.max(targetH, Globals.slidesV4Thresh + 2));
                }

                if (slides.getLength() > Globals.slidesV4Thresh) {
                    state = State.FINISH_DEPOSIT;
                }
                break;
            case FINISH_DEPOSIT:
                startDeposit = false;
                slides.setTargetLength(targetH);
                endAffector.v4Servo.setTargetAngle(v4BarDepositAngle,v4ServoPower);
                endAffector.topServo.setTargetAngle(topServoDepositAngle,topServoPower);

                if (endAffector.v4Servo.inPosition() && slides.inPosition(5)) {
                    state = State.DEPOSIT;
                }
                break;
            case DEPOSIT:
                slides.setTargetLength(targetH);

                if (inPixelAdjustmentMode) {
                    alreadySwitched = true;
                    endAffector.v4Servo.setTargetAngle(v4BarReadjustAngle,v4ServoPower);
                    endAffector.topServo.setTargetAngle(topServoReadjustAngle,topServoPower);
                } else {
                    if (alreadySwitched) {
                        endAffector.v4Servo.setTargetAngle(v4BarDepositAngle,v4ServoPower);
                        endAffector.topServo.setTargetAngle(topServoDepositAngle,topServoPower);
                    } else {
                        endAffector.v4Servo.setTargetAngle(v4BarDepositAngle,v4ServoPower);
                        endAffector.topServo.setTargetAngle(topServoDepositAngle,topServoPower);
                    }
                }

                if (release.readyToRetract()) {
                    beginRetractTime = System.currentTimeMillis();
                    state = State.START_RETRACT;
                }
                break;
            case START_RETRACT:
                inPixelAdjustmentMode = false;
                alreadySwitched = false;
                isSlowSlidesMode = false;
                if (System.currentTimeMillis() - beginRetractTime > 200) {
                    release.close();

                    if (endAffector.v4Servo.getCurrentAngle() <= Math.toRadians(135)) {
                        slides.setTargetLength(Globals.slidesV4Thresh + 2);
                        if (endAffector.v4Servo.getCurrentAngle() <= Math.toRadians(90)) {
                            endAffector.topServo.setTargetAngle(topServoTransferAngle, topServoPower);
                        } else {
                            endAffector.topServo.setTargetAngle(0, topServoPower);
                        }
                    } else {
                        slides.setTargetLength(targetH + 3); // TODO: Fix this to make the arm wait for slides to go up then move
                    }

                    if (endAffector.v4Servo.inPosition()) {
                        state = State.RETRACT;
                    }
                } else {
                    endAffector.v4Servo.setTargetAngle(v4BarTransferAngle, v4ServoPower);
                    endAffector.topServo.setTargetAngle(topServoDepositAngle, topServoPower);
                }
                break;
            case RETRACT:
                slides.setTargetLength(0.0);

                if (slides.inPosition(2)) {
                    state = State.INTAKE;
                }
                break;
            case IDLE: // We are boring :(
                break;
        }
        slides.update();
        release.update();
    }

    public boolean inPixelAdjustmentMode = false;
    boolean alreadySwitched = false;

    public void togglePixelAdjustmentMode() {
        inPixelAdjustmentMode = !inPixelAdjustmentMode;
    }
}