package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

@Autonomous(name = "BLUE Cycle Auto Up")
public class CycleAutoBlueUp extends LinearOpMode {
    private Vision vision;
    protected TeamPropDetectionPipeline.TeamPropLocation teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
    protected Robot robot;

    private long start;
    private Vector3 deposit = null;
    private Pose2d boardPreload = null;
    public static double fx = 0.15;
    public static double fy = 0.15;
    public static double fh = 0.01; // JANK

    private int numCycles = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.isRed = false;
        try {
            doInitialization();
            waitForStart();
            Globals.autoStartTime = System.currentTimeMillis();
            pause(3100); // todo pause for overcharged !!

            doGroundPreload();
            doBoardPreload();

            for (int i = 0; i < numCycles; i++) {
                navigateBackToStack();
                intakeStack();
                if (System.currentTimeMillis() - Globals.autoStartTime > 28000) { // if we have less than 2 secs remaining we don't go for the deposit
                    break;
                }
                depositOnBoard();
            }
            pause(150);
            end();
        } catch (Error e) {
            Log.e("*******************ERROR*******************", e + "");
        }
    }

    public void doInitialization() {
        Globals.RUNMODE = RunMode.AUTO;
        Globals.NUM_PIXELS = 1;

        vision = new Vision(hardwareMap, telemetry, false, true, true);
        robot = new Robot(hardwareMap, vision);

        robot.droppers.leftDown();
        robot.droppers.rightRelease();

        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        robot.hang.quickTurnOnOff();

        while (opModeInInit() && !isStopRequested()) {
            robot.drivetrain.setPoseEstimate(new Pose2d(11.804, 60.75, Math.PI / 2));

            teamPropLocation = vision.teamPropDetectionPipeline.getTeamPropLocation();
            vision.teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);
            robot.deposit.release.preGrab();

            robot.update();
        }
        Log.e("team prop location", teamPropLocation + "");
        vision.disableTeamProp();
        vision.enableAprilTag();

        robot.intake.actuationFullyUp();
    }

    /**
     * <b>Do preload operation</b> <br>
     * <br>
     * Go to preload position and deposit it
     */

    double intakeYDistance;

    public void doGroundPreload() {
        Pose2d groundPreloadPosition = robot.drivetrain.getPoseEstimate();

        Log.e("teamPropLocation", teamPropLocation + "");
        deposit = new Vector3(5, 0, 7.75);

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition = new Pose2d(14.5, 46, Math.PI/2);
                boardPreload =          new Pose2d(48.35, 41, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                robot.goToPoint(new Pose2d(23, 36.5, Math.toRadians(110)), this, false, false);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(21, 38.5, Math.PI), this, false, false);
                intakeYDistance = 35;
                intakeXDistances = new double[] {-55.2, -55.2, -55.2, -55.2, -55.2};
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(16, 34, Math.PI/2);
                boardPreload =          new Pose2d(48.15, 33.5, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                releaseAndTriggerDeposit();
                robot.goToPoint(new Pose2d(16, 39, Math.PI/2), this, false, false);
                robot.goToPoint(new Pose2d(15, 39, 2*Math.PI/3), this, false, false);
                intakeYDistance = 35.5;
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(11.8, 46.5, Math.PI/2);
                boardPreload =          new Pose2d(47.25, 28, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                robot.goToPoint(new Pose2d(11, 32.25, Math.toRadians(50)), this, false, false);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(24, 27, Math.PI), this, false, false);
                intakeYDistance = 35.5;
                intakeXDistances = new double[] {-54.4, -54.4, -54.4, -55.2, -55.2};
                break;
        }
    }

    public void releaseAndTriggerDeposit() {
        robot.droppers.leftRelease();
        pause(150);

        robot.deposit.depositAt(deposit); // async call to deposit
    }

    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */

    int pixelIndex = 4; // 0 index based
    double[] intakeXDistances = new double[] {-54.8, -54.8, -54.8, -54.8, -54.8}; // 1 <-- 5 pixels

    public void doBoardPreload() {
        robot.goToPointWithLimitSwitch(boardPreload, this, true, 0.8);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseOne();
    }

    public void navigateBackToStack() {

        robot.intake.actuationFullyUp(); // non destructive

        robot.goToPoint(new Pose2d(29.25, 59, Math.PI), this, false, false);
        robot.goToPoint(new Pose2d(-42, 59, Math.PI), this, false, false);
        robot.goToPoint((new Pose2d(intakeXDistances[pixelIndex] + 3, intakeYDistance , Math.PI)), this, false, true);
        robot.intake.on();
        robot.intake.setActuationHeight(pixelIndex, 0.85);
        robot.goToPoint((new Pose2d(intakeXDistances[pixelIndex], intakeYDistance , Math.PI)), this, true, true);
    }

    public void intakeStack() { //todo decrease initial pixelIndex for match with overcharged
        Globals.mergeUltrasonics = true;
        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.on();

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 650) {
            robot.update();
        }

        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.setActuationHeight(pixelIndex, 0.85) ;

        start = System.currentTimeMillis();
        if (pixelIndex < 1 && Globals.NUM_PIXELS != 2) {
            robot.intake.setActuationHeight(0, 1.0);

            if (teamPropLocation != TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
                robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance - 12, Math.PI), this, false, false, 1.0);
                if (Globals.NUM_PIXELS == 2) return;
                robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance, Math.PI), this, false, false, 1.0);
            } else {
                robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance-1, Math.PI), this, false, false, 1.0);
                if (Globals.NUM_PIXELS == 2) return;
                robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance + 4, Math.PI), this, false, false, 1.0);
            }
        } else {
            while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 850) {
                robot.intake.setActuationHeight(0, 0.1);
                robot.update();
            }
        }

        if (robot.intake.reversed) {
            Log.e("trying to turn intake on", "turning");
            robot.intake.on();
            pause(1000);
        }
        robot.intake.reversed = false;

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 100) {
            Intake.reversedPower = -0.1;
            robot.intake.reverse();
            robot.update();
        }

        Globals.NUM_PIXELS = 2;
        Globals.mergeUltrasonics = false;
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */

    int cycleNum = 0;
    double[] depositHeights = new double[] {12, 14};

    public void depositOnBoard() {
        deposit.z = depositHeights[cycleNum];
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT) {
            deposit.z = deposit.z + 3.5;
        }

        robot.intake.actuationFullyUp();
        robot.goToPoint(new Pose2d(-44, 58, Math.PI), this, false, false);

        robot.goToPointWithDepositMOVING(new Pose2d(32, 58, Math.PI), this, false, false, 0);
        robot.deposit.depositAt(deposit); // async call
        robot.goToPoint(new Pose2d(39, 41, Math.PI), this, false, false);
        robot.intake.off();

        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
            robot.goToPointWithLimitSwitch(new Pose2d(46.85, 41, Math.PI), this, true, 0.3);
        }
        else {
            robot.goToPointWithLimitSwitch(new Pose2d(46.75, 41, Math.PI), this, true, 0.3);
        }

        robot.releaseOne();
        sleep(150);
        robot.releaseOne();
        cycleNum++;
    }

    /**
     * Assumes that it is in the parking line row
     */
    public void end() {
        robot.deposit.retract();
        robot.deposit.release.intake();

        robot.drivetrain.forceStopAllMotors();
        Log.e("time spent", (System.currentTimeMillis()-start) + "");
        Globals.AUTO_ENDING_POSE = robot.drivetrain.getPoseEstimate();

        pause(1500);
    }

    public void pause (double milliseconds) {
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds && opModeIsActive()) {
            robot.update();
        }
    }
}
