package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.airplane.Airplane;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.droppers.Droppers;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Func;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    public HardwareQueue hardwareQueue;

    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final Deposit deposit;
    public final Intake intake;
    public final Airplane airplane;
    public final Hang hang;
    public final Droppers droppers;
    public final Vision vision;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Vision vision) {
        hardwareQueue = new HardwareQueue();
        this.vision = vision;
        Globals.autoStartTime  = -1;

        sensors = new Sensors(hardwareMap, hardwareQueue, this);

        if (vision != null) {
            drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors, vision, this);
        } else {
            drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors, this);
        }

        deposit = new Deposit(hardwareMap, hardwareQueue, sensors, this);
        intake = new Intake(hardwareMap, hardwareQueue, sensors, this);
        airplane = new Airplane(hardwareMap, hardwareQueue);
        hang = new Hang(hardwareMap, hardwareQueue);
        droppers = new Droppers(hardwareMap, hardwareQueue);

        TelemetryUtil.setup();
    }

    public void update() {
        START_LOOP();
        Globals.gotBloodyAnnihilated = System.currentTimeMillis() - Globals.autoStartTime > 29500 && Globals.autoStartTime != -1 && deposit.slides.length > 1;
        updateSubsystems();
        updateTelemetry();
    }

    private void updateSubsystems() {
        hardwareQueue.update();
        sensors.update();

        drivetrain.update();
        deposit.update();
        intake.update();
        airplane.update();
        hang.update();
        droppers.update();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
    }

    public void goToPoint(Pose2d pose, Func func, boolean finalAdjustment, boolean stop, double maxPower) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, stop, maxPower); // need this to start the process so thresholds don't immediately become true
        update(); // maybe remove?
        while(((boolean) func.call()) && System.currentTimeMillis() - start <= 5000 && drivetrain.isBusy()) {
            update();
        }
    }

    public void followSpline(Spline spline, Func func) {
        long start = System.currentTimeMillis();
        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        drivetrain.setMaxPower(1);
        update();

        do {
            update();
        } while (((boolean) func.call()) && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }

    double movingSlidesDepositHeight = 10.25;
    public void followSplineWithIntakeAndDeposit(Spline spline, Vector3 depositVector3, double depositTriggerThreshold, double intakeReverseThreshold, double maxPower, boolean finalAdjustment, boolean stop) {
        long start = System.currentTimeMillis();
        drivetrain.setFinalAdjustment(finalAdjustment);
        drivetrain.setStop(stop);
        drivetrain.setMaxPower(maxPower);

        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        update();

        do {
            intake.actuationFullyUp();

            if (drivetrain.getPoseEstimate().x > depositTriggerThreshold) {
                intake.off();
                deposit.depositAt(new Vector3(5, 0, movingSlidesDepositHeight));
            }

            if (drivetrain.localizers[0].getPoseEstimate().x > intakeReverseThreshold && drivetrain.localizers[0].getPoseEstimate().x < depositTriggerThreshold) {
                intake.reverse();
            }

            update();
        } while (System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }

    public void followSplineWithIntake(Spline spline, double intakeStopThreshold, double intakeReverseThreshold, double maxPower, boolean finalAdjustment, boolean stop) {
        long start = System.currentTimeMillis();
        drivetrain.setFinalAdjustment(finalAdjustment);
        drivetrain.setStop(stop);
        drivetrain.setMaxPower(maxPower);

        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        update();

        do {
            intake.actuationFullyUp();

            if (drivetrain.getPoseEstimate().x > intakeStopThreshold) {
                intake.off();
            }

            if (drivetrain.localizers[0].getPoseEstimate().x > intakeReverseThreshold && drivetrain.localizers[0].getPoseEstimate().x < intakeStopThreshold) {
                intake.reverse();
            }

            update();
        } while (System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy());
    }

    public void followSplineWithIntakeAndDepositAndUltrasonicCheck(Spline spline, Vector3 depositVector3, double depositTriggerThreshold, double intakeReverseThreshold, double maxPower, boolean finalAdjustment, boolean stop, double beginCornerUltrasonicCheckThreshold, double endCornerUltrasonicCheckThreshold) {
        drivetrain.setFinalAdjustment(finalAdjustment);
        drivetrain.setStop(stop);
        drivetrain.setMaxPower(maxPower);

        drivetrain.setPath(spline);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        update();

        do {
            intake.actuationFullyUp();

            if (drivetrain.localizers[0].getPoseEstimate().x > depositTriggerThreshold) {
                intake.off();
                deposit.depositAt(new Vector3(5, 0, movingSlidesDepositHeight)); // async call to deposit
            }

            if (drivetrain.localizers[0].getPoseEstimate().x > intakeReverseThreshold && drivetrain.localizers[0].getPoseEstimate().x < depositTriggerThreshold) {
                intake.reverse();
            }

//            if (sensors.isDepositTouched() && deposit.state == Deposit.State.DEPOSIT) { //break out of spline if touch detected
//                drivetrain.setPath(null);
//                drivetrain.state = Drivetrain.State.BRAKE;
//                break;
//            }

            if (drivetrain.getPoseEstimate().x > beginCornerUltrasonicCheckThreshold) {
                if (drivetrain.getPoseEstimate().x < endCornerUltrasonicCheckThreshold) {
                    drivetrain.useUltrasonicDetection = true;
                } else {
                    drivetrain.useUltrasonicDetection = false;
                }
            }
            update();
        } while (drivetrain.isBusy());
        drivetrain.useUltrasonicDetection = false;
    }

    public void splineToPoint(Pose2d pose, Func func, boolean finalAdjustment, boolean stop, double maxPower, boolean isReversed) {
        Pose2d clonedPose = pose.clone();
        drivetrain.setFinalAdjustment(finalAdjustment);
        drivetrain.setStop(stop);
        drivetrain.setMaxPower(maxPower);

        Spline path = new Spline(drivetrain.getPoseEstimate(), 3);
        path.setReversed(isReversed);

        path.addPoint(clonedPose);

        followSpline(path, func);
    }

    public void splineToPoint(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, double maxPower, boolean isReversed) {
        splineToPoint(pose, opMode::opModeIsActive, finalAdjustment, stop, maxPower, isReversed);
    }

    public void goToPoint(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, double maxPower) {
        goToPoint(pose, opMode::opModeIsActive, finalAdjustment, stop, maxPower);
    }

    public void goToPoint(Pose2d pose, Func func, boolean stop, double maxPower, double finalXThreshold, double finalYThreshold, double finalHeadingThreshold) {
        double tx = Drivetrain.finalXThreshold;
        double ty = Drivetrain.finalYThreshold;
        double th = Drivetrain.finalTurnThreshold;
        Drivetrain.finalXThreshold = finalXThreshold;
        Drivetrain.finalYThreshold = finalYThreshold;
        Drivetrain.finalTurnThreshold = finalHeadingThreshold;
        goToPoint(pose, func, true, stop, maxPower);
        Drivetrain.finalXThreshold = tx;
        Drivetrain.finalYThreshold = ty;
        Drivetrain.finalTurnThreshold = th;
    }

    public void splineToPoint(Pose2d pose, LinearOpMode opmode, boolean isReversed) {
        splineToPoint(pose, opmode, false, true, isReversed);
    }

    public void splineToPoint(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, boolean isReversed) {
        splineToPoint(pose, opMode, finalAdjustment, stop, 1.0, isReversed);
    }

    public void goToPoint(Pose2d pose, LinearOpMode opMode) {
        goToPoint(pose, opMode, false, true);
    }

    public void goToPoint(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop) {
        goToPoint(pose, opMode, finalAdjustment, stop, 1.0);
    }

    public void goToPointWithLimitSwitch(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, double power) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, false, power); // need this to start the process so thresholds don't immediately become true
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy()) {
            if (sensors.isDepositTouched()) { //break out of spline if touch detected
                drivetrain.setPath(null);
                drivetrain.state = Drivetrain.State.BRAKE;
                update();
                break;
            }
            update();
        }
    }

    public void goToPointWithDeposit(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, Vector3 depositVector3, double xThreshold) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, stop, 1.0); // need this to start the process so thresholds don't immediately become true
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy()) {
            if (drivetrain.localizers[0].getPoseEstimate().x > xThreshold) {
                deposit.depositAt(depositVector3); // async call to deposit
            }
            update();
        }
    }

    public void goToPointWithDepositMOVING(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, double xThreshold) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, stop, 1.0); // need this to start the process so thresholds don't immediately become true
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy()) {
            if (drivetrain.localizers[0].getPoseEstimate().x > xThreshold) {
                deposit.depositAt(new Vector3(5, 0, movingSlidesDepositHeight)); // async call to deposit
            }
            update();
        }
    }

    public void goToPointWithDeposit(Pose2d pose, Func func, boolean finalAdjustment, boolean stop, Vector3 depositVector3, double xThreshold) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, stop, 1.0); // need this to start the process so thresholds don't immediately become true
        while(((boolean) func.call()) && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy()) {
            if (drivetrain.localizers[0].getPoseEstimate().x > xThreshold) {
                deposit.depositAt(depositVector3); // async call to deposit
            }
            update();
        }
    }

    public void followSpline(Spline spline, Func func, boolean finalAdjustment, boolean stop) {
        drivetrain.setFinalAdjustment(finalAdjustment);
        drivetrain.setStop(stop);
        followSpline(spline, func);
    }

    public void goToPointWithDepositAndIntake(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, Vector3 depositVector3, double depositThreshold, double reverseIntakeThreshold) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment, stop, 1.0); // need this to start the process so thresholds don't immediately become true
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy()) {
            intake.actuationFullyUp();
            if (drivetrain.localizers[0].getPoseEstimate().x > depositThreshold) {
                intake.off();
                deposit.depositAt(new Vector3(5, 0, movingSlidesDepositHeight)); // async call to deposit
            }

            if (drivetrain.localizers[0].getPoseEstimate().x > reverseIntakeThreshold) {
                intake.reverse();
            }
            update();
        }
    }

    public void splineToPointWithDepositAndIntake(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment, boolean stop, boolean isReversed, Vector3 depositVector3, double xThreshold) {
        long start = System.currentTimeMillis();
        Spline path = new Spline(drivetrain.getPoseEstimate(), 3);
        path.setReversed(isReversed);

        path.addPoint(pose.clone());

        drivetrain.setPath(path);
        drivetrain.state = Drivetrain.State.GO_TO_POINT;
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start <= 10000 && drivetrain.isBusy()) {
            if (drivetrain.localizers[0].getPoseEstimate().x > xThreshold) {
                intake.reverse();
                deposit.depositAt(depositVector3); // async call to deposit
            }
            update();
        }
    }

    public void alignWithStack(LinearOpMode opMode, Pose2d stackPose, double intakeOffset, double maxPower) {
        drivetrain.startStackAlignment(stackPose, intakeOffset, maxPower);
        while (opMode.opModeIsActive() && drivetrain.isBusy()) {
            update();
        }
    }

    public void depositAt(double targetH, double targetX) {
        deposit.depositAt(targetH, targetX);
        update();
        while (deposit.state != Deposit.State.DEPOSIT && !deposit.slides.inPosition(1.0)) {
            update();
        }
    }

    public void releaseOne() {
        deposit.releaseOne();
        update();
        while (deposit.release.isBusy()) {
            drivetrain.forceStopAllMotors(); // not sure if this is good
            update();
        }
    }

    public void releaseTwo() {
        deposit.releaseOne();
        update();
        while (deposit.release.isBusy()) {
            drivetrain.forceStopAllMotors(); // not sure if this is good
            update();
        }
        update();
        deposit.releaseOne();
        while (deposit.release.isBusy()) {
            drivetrain.forceStopAllMotors(); // not sure if this is good
            update();
        }
    }

    public void checkForPartner(LinearOpMode opmode, double checkTime) {
        long start = System.currentTimeMillis();
        while (opmode.opModeIsActive() && System.currentTimeMillis() - start < checkTime ||
                (sensors.ultrasonicCheckState == Sensors.UltrasonicCheckState.WAIT || sensors.ultrasonicCheckState == Sensors.UltrasonicCheckState.CONFIRM_BLOCKED)) {
            update();
            drivetrain.forceStopAllMotors();
            sensors.checkForPartner();
            Log.e("ultrasonic state", sensors.ultrasonicCheckState + "");
        }
    }}
