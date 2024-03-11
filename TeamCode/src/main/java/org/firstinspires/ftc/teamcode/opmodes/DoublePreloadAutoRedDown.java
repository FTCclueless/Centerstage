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

@Autonomous(name = "RED Double Preload Auto Down")
public class DoublePreloadAutoRedDown extends LinearOpMode {
    private Vision vision;
    protected TeamPropDetectionPipeline.TeamPropLocation teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
    protected Robot robot;

    private long start;
    private Vector3 deposit = null;
    private Pose2d boardPreload = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.isRed = true;
        try {
            doInitialization();
            waitForStart();

            pause(17000);

            doGroundPreload();
            navigateToBoard();
            doBoardPreload();

            park();
        } catch (Error e) {
            Log.e("*******************ERROR*******************", e + "");
        }
    }

    /**
     * <b>brief:</b> Does initialization for potential auto <br>
     * <br>
     * <ul>
     *     <li>Instantiates robot</li>
     *     <li>Sets Globals.RUNMODE</li>
     *     <li>Defines the reflect variable for your own usage</li>
     *     <li>Sets drivetrain initial position</li>
     *     <li>Does vision init</li>
     * </ul>
     */
    public void doInitialization() {
        Globals.RUNMODE = RunMode.AUTO;
        Globals.NUM_PIXELS = 1;

        vision = new Vision(hardwareMap, telemetry, true, true, true);
        robot = new Robot(hardwareMap, vision);

        robot.droppers.leftDown();
        robot.droppers.rightRelease();

        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        robot.hang.quickTurnOnOff();

        while (opModeInInit() && !isStopRequested()) {
            teamPropLocation = vision.teamPropDetectionPipeline.getTeamPropLocation();
            vision.teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);
            robot.deposit.release.preGrab();
            robot.drivetrain.setPoseEstimate(new Pose2d(-35.411, -59.75, -Math.PI / 2));

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
        Pose2d groundPreloadPosition;

        Log.e("teamPropLocation", teamPropLocation + "");

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition = new Pose2d(-36.25, -42.5, -Math.PI/2);
                boardPreload =          new Pose2d(46.5, -29, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-45.5, -37.5, -Math.toRadians(60)), this, false, true);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-36.25, -34.5, -Math.PI/2);
                boardPreload =          new Pose2d(46.5, -33.75, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(-36.911, -51, -Math.PI/2);
                boardPreload =          new Pose2d(46.5, -41.75, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-33.5, -33.5, -Math.toRadians(125)), this, false, true);
                break;
        }
        robot.droppers.leftRelease();

        pause(150);
    }

    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */
    public void navigateToBoard() {
        deposit = new Vector3(5, 0, 10);

        switch (teamPropLocation) {
            case LEFT:
                robot.goToPoint(new Pose2d(-41.5, -40.5, -Math.toRadians(60)), this, false, true, 1);
                robot.goToPoint(new Pose2d(-41.5, -40, -Math.PI/2), this, false, true, 1);
                robot.goToPoint(new Pose2d(-36.5, -40, -Math.PI/2), this, false, true, 1);
                robot.goToPoint(new Pose2d(-36.5, -11.5, -Math.PI/2), this, false, false, 1);
                robot.goToPoint(new Pose2d(-36.5, -11.5, Math.PI), this, false, false, 1);

                deposit = new Vector3(5, 0, 10.25);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-51.5, -8.5, Math.PI), this, false, false);

                deposit = new Vector3(5, 0, 10);
                break;
            case RIGHT:
                // Jank -- eric
                robot.goToPoint(new Pose2d(-37, -7, Math.PI), this, false, false);

                deposit = new Vector3(5, 0, 10.25);
                break;
        }

        // JANK
        robot.goToPointWithDeposit(new Pose2d(30, -8.2, Math.PI), this, false, false, deposit,0);
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        robot.goToPoint(new Pose2d(boardPreload.x-7, boardPreload.y, boardPreload.heading), this, false, false);

        robot.goToPoint(boardPreload, this::opModeIsActive, true, 0.75, 0.75,1.1, 2.5);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseTwo();
    }

    /**
     * Assumes that it is in the parking line row
     */
    public void park() {
        robot.goToPoint(new Pose2d(42, -9, Math.PI), this, false, false); // intermediate parking
        robot.goToPoint(new Pose2d(58, -9, Math.PI), this, false, true); // parking

        Globals.AUTO_ENDING_POSE = robot.drivetrain.getPoseEstimate();

        pause (500);
    }

    public void pause (double milliseconds) {
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds && opModeIsActive()) {
            robot.update();
        }
    }
}
