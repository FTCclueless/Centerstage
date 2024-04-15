package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
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

    public void doGroundPreload() {
        Pose2d groundPreloadPosition = robot.drivetrain.getPoseEstimate();

        Log.e("teamPropLocation", teamPropLocation + "");
        deposit = new Vector3(5, 0, 7.00);

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition = new Pose2d(13.5, 46.5, Math.PI/2);
                boardPreload =          new Pose2d(48.35, 40.5, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                robot.goToPoint(new Pose2d(20.5, 35.5, Math.toRadians(125)), this, false, false);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(21, 37.5, Math.PI), this, false, false);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(16, 34, Math.PI/2);
                boardPreload =          new Pose2d(48.35, 33, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                releaseAndTriggerDeposit();
                robot.goToPoint(new Pose2d(16, 39, Math.PI/2), this, false, false);
                robot.goToPoint(new Pose2d(15, 39, 2*Math.PI/3), this, false, false);
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(11.8, 46.5, Math.PI/2);
                boardPreload =          new Pose2d(47.25, 28, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                robot.goToPoint(new Pose2d(10, 30, Math.toRadians(50)), this, false, false);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(24, 27, Math.PI), this, false, false);
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
    double[] intakeXDistances = new double[] {-54.5, -55, -55.2, -55.2, -54}; // 1 <-- 5 pixels

    public void doBoardPreload() {
        robot.goToPointWithLimitSwitch(boardPreload, this, true, 1.0);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseOne();
    }

    double intakeYDistance = 34.5;

    public void navigateBackToStack() {
        robot.intake.on();

        robot.intake.actuationFullyUp(); // non destructive

        robot.goToPoint(new Pose2d(29.25, 58, Math.PI), this, false, false);
        robot.goToPoint(new Pose2d(-42, 58, Math.PI), this, false, false);
        robot.intake.setActuationHeight(pixelIndex, 1.0);
        robot.goToPoint((new Pose2d(intakeXDistances[pixelIndex], intakeYDistance , Math.PI)), this, false, true);
    }

    public void intakeStack() {
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
            robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance-15, Math.PI), this, false, false, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance, Math.PI), this, false, false, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[0], intakeYDistance, Math.PI), this, false, false, 1.0);
        } else {
            while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 850) {
                robot.intake.setActuationHeight(0, 0.04);
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
        while (System.currentTimeMillis() - start < 650) {
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
    double[] depositHeights = new double[] {13, 15};

    public void depositOnBoard() {
        deposit.z = depositHeights[cycleNum];
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT) {
            deposit.z = deposit.z + 5;
        }

        robot.intake.actuationFullyUp();
        robot.goToPoint(new Pose2d(-42, 56, Math.PI), this, false, false);

        robot.goToPoint(new Pose2d(30, 56, Math.PI), this, false, false);
        robot.deposit.depositAt(deposit); // async call
        robot.goToPoint(new Pose2d(40, 40, Math.PI), this, false, false);
        robot.intake.off();

        robot.goToPointWithLimitSwitch(new Pose2d(48.5,40, Math.PI), this, false, 1.0);

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
