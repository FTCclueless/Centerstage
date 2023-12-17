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
    protected boolean red = false;
    protected int reflect = 1;

    private long start;
    private Vector3 boardPreloadDeposit = null;

    @Override
    public void runOpMode() throws InterruptedException {
        doInitialization();

        waitForStart();
        doGroundPreload();
        doBoardPreload();
        park();
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

        Pose2d targetBoard = AutoPathConstants.targetBoard.clone();
        targetBoard.y *= reflect;
        robot.deposit.setTargetBoard(targetBoard);

        if (red) {
            robot.droppers.leftDown();
            robot.droppers.rightRelease();
        } else {
            robot.droppers.rightDown();
            robot.droppers.leftRelease();
        }

        robot.hangActuation.up();
        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        // TODO: Add disable vision flag in case of complications :)
        while (opModeInInit()) {
            teamPropLocation = vision.teamPropDetectionPipeline.getTeamPropLocation();
            vision.teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);
            robot.deposit.dunker.lock();

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
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT && !red) {
            teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.RIGHT;
        } else if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT && !red) {
            teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.LEFT;
        }

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition.x += AutoPathConstants.groundPreloadLeftOffset.x;
                groundPreloadPosition.y += AutoPathConstants.groundPreloadLeftOffset.y * reflect;
                groundPreloadPosition.heading += (red ? AutoPathConstants.groundPreloadLeftOffset.heading : AutoPathConstants.blueGroundPreloadRightOffset.x);
                boardPreloadDeposit = AutoPathConstants.boardPreloadLeftDeposit.clone();
                if (!red)
                    boardPreloadDeposit = AutoPathConstants.blueBoardPreloadRightDeposit.clone();
                break;
            case CENTER:
                groundPreloadPosition.x += AutoPathConstants.groundPreloadCenterOffset.x;
                groundPreloadPosition.y += AutoPathConstants.groundPreloadCenterOffset.y * reflect;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadCenterOffset.heading;
                boardPreloadDeposit = AutoPathConstants.boardPreloadCenterDeposit.clone();
                if (!red)
                    boardPreloadDeposit = AutoPathConstants.blueBoardPreloadCenterDeposit.clone();
                break;
            case RIGHT:
                groundPreloadPosition.x += (red ? AutoPathConstants.groundPreloadRightOffset.x : AutoPathConstants.blueGroundPreloadLeftOffset.x);
                groundPreloadPosition.y += AutoPathConstants.groundPreloadRightOffset.y * reflect;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadRightOffset.heading;
                boardPreloadDeposit = AutoPathConstants.boardPreloadRightDeposit.clone();
                if (!red)
                    boardPreloadDeposit = AutoPathConstants.blueBoardPreloadLeftDeposit.clone();
                break;
        }

        robot.goToPoint(groundPreloadPosition, this);

        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT && red) {
            robot.goToPoint(new Pose2d(robot.drivetrain.getPoseEstimate().x - AutoPathConstants.groundPreloadStrafeOffset, robot.drivetrain.getPoseEstimate().y, robot.drivetrain.getPoseEstimate().heading), this);
        }

        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT && !red) {
            robot.goToPoint(new Pose2d(robot.drivetrain.getPoseEstimate().x - AutoPathConstants.groundPreloadStrafeOffset, robot.drivetrain.getPoseEstimate().y, robot.drivetrain.getPoseEstimate().heading), this);
        }

        start = System.currentTimeMillis();
        if (red) {
            robot.droppers.leftRelease();
        } else {
            robot.droppers.rightRelease();
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



        robot.deposit.depositAt(boardPreloadDeposit.z, boardPreloadDeposit.y, boardPreloadDeposit.x);

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 3000) {
            robot.update();
        }
        robot.dunk();
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
            robot.update();
        }
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
