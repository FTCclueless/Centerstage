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

/**
 * Auto that just parks
 *
 * TODO: There is no blue side support
 */

@Autonomous(name = "Double Preload Auto")
public class DoublePreloadAuto extends LinearOpMode {
    private Vision vision;
    protected TeamPropDetectionPipeline.TeamPropLocation teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
    protected Robot robot;

    protected boolean up = true;
    protected boolean red = true;
    protected int reflect = 1;

    private long start;
    private Vector3 boardPreloadDeposit = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            doInitialization();

            waitForStart();
            doGroundPreload();
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

        vision = new Vision(hardwareMap, telemetry, red, true, true);
        robot = new Robot(hardwareMap, vision);
        reflect = red ? 1 : -1;

        if (up) {
            Pose2d startPos = AutoPathConstants.startUp.clone();
            startPos.y *= reflect;
            startPos.heading *= reflect;
            robot.drivetrain.setPoseEstimate(startPos);
        } else {
            Pose2d startPos = AutoPathConstants.startDown.clone();
            startPos.y *= reflect;
            startPos.heading *= reflect;
            robot.drivetrain.setPoseEstimate(startPos);
        }

        if (red) {
            robot.droppers.rightDown();
            robot.droppers.leftRelease();
        } else {
            robot.droppers.leftDown();
            robot.droppers.rightRelease();
        }

        robot.hangActuation.up();
        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        // TODO: Add disable vision flag in case of complications :)
        while (opModeInInit()) {
            teamPropLocation = vision.teamPropDetectionPipeline.getTeamPropLocation();
            vision.teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);
            robot.deposit.release.close();

            robot.update();
        }
        Log.e("team prop location", teamPropLocation + "");
        vision.disableTeamProp();
        vision.enableAprilTag();
    }

    /**
     * <b>Do preload operation</b> <br>
     * <br>
     * Go to preload position and deposit it
     */

    public void doGroundPreload() {
        Pose2d groundPreloadPosition = robot.drivetrain.getPoseEstimate();

        Log.e("teamPropLocation", teamPropLocation + "");

        // Convert because monkey coding
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT && red) {
            teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.RIGHT;
        } else if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT && red) {
            teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.LEFT;
        }

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition.x += (red ? AutoPathConstants.groundPreloadLeftOffset.x : AutoPathConstants.blueGroundPreloadRightOffset.x);
                groundPreloadPosition.y += AutoPathConstants.groundPreloadLeftOffset.y * reflect;
                groundPreloadPosition.heading += (red ? AutoPathConstants.groundPreloadLeftOffset.heading : AutoPathConstants.blueGroundPreloadLeftOffset.heading);
                break;
            case CENTER:
                groundPreloadPosition.x += AutoPathConstants.groundPreloadCenterOffset.x;
                groundPreloadPosition.y += AutoPathConstants.groundPreloadCenterOffset.y * reflect;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadCenterOffset.heading;
                break;
            case RIGHT:
                groundPreloadPosition.x += (red ? AutoPathConstants.groundPreloadRightOffset.x : AutoPathConstants.blueGroundPreloadLeftOffset.x);
                groundPreloadPosition.y += AutoPathConstants.groundPreloadRightOffset.y * reflect;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadRightOffset.heading;
                break;
        }

        boardPreloadDeposit = AutoPathConstants.boardPreloadDeposit.clone();

        if (!red && teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
            Log.e("good", "e");
            robot.goToPoint(new Pose2d(groundPreloadPosition.x, groundPreloadPosition.y + 5, groundPreloadPosition.heading), this);
        }
        else {
            robot.goToPoint(groundPreloadPosition, this);
        }

        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT && red) {
            robot.goToPoint(new Pose2d(robot.drivetrain.getPoseEstimate().x - AutoPathConstants.groundPreloadStrafeOffset, robot.drivetrain.getPoseEstimate().y, robot.drivetrain.getPoseEstimate().heading), this);
        }


        start = System.currentTimeMillis();
        if (red) {
            robot.droppers.rightRelease();
        } else {
            robot.droppers.leftRelease();
        }

        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT && !red) {
            //robot.goToPoint(new Pose2d(robot.drivetrain.getPoseEstimate().x - AutoPathConstants.groundPreloadStrafeOffset, robot.drivetrain.getPoseEstimate().y, robot.drivetrain.getPoseEstimate().heading), this);
        }

        while (System.currentTimeMillis() - start < 400) {
            robot.update();
        }
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        Pose2d boardPreload = AutoPathConstants.boardPreload.clone();
        boardPreload.y *= reflect;

        Pose2d initialIntake = AutoPathConstants.initialIntake.clone();
        initialIntake.y *= reflect;

        // Up behavior is to instantly deposit preload on board
        if (!up)
            robot.goToPoint(initialIntake, this);
            // TODO: Intake

        robot.goToPoint(boardPreload, this);

        robot.depositAt(boardPreloadDeposit.z, boardPreloadDeposit.y);

        robot.releaseTwo();
    }

    /**
     * Assumes that it is in the parking line row
     */
    public void park() {
        Pose2d park = AutoPathConstants.parkingLocation.clone();
        park.y *= reflect;

        Pose2d parkIntermediate = AutoPathConstants.intermediateParkingLocation.clone();
        parkIntermediate.y *= reflect;

        robot.goToPoint(parkIntermediate, this);
        robot.goToPoint(park, this);

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 750) {
            robot.update();
        }

        Globals.AUTO_ENDING_POSE = robot.drivetrain.getPoseEstimate();
    }
}
