package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.droppers.Droppers;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.apriltags.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

/**
 * Auto that just parks
 *
 * TODO: There is no blue side support
 */

@Autonomous(name = "Double Preload Auto")
public class DoublePreloadAuto extends LinearOpMode {
    private Vision vision = new Vision();
    private TeamPropDetectionPipeline teamPropDetectionPipeline;
    protected TeamPropDetectionPipeline.TeamPropLocation teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
    protected Robot robot;

    protected boolean up = true;
    protected boolean red = true;
    protected int reflect = 1;

    long start;

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

        robot = new Robot(hardwareMap);
        reflect = red ? 1 : -1;

        if (up) {
            Pose2d startPos = AutoPathConstants.startUp.clone();
            startPos.y *= reflect;
            startPos.heading*= reflect;
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
        } else {
            robot.droppers.rightDown();
        }

        initVision();

        // TODO: Add disable vision flag in case of complications :)
        while (opModeInInit()) {
            teamPropLocation = teamPropDetectionPipeline.getTeamPropLocation();
            teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);

            robot.update();
        }
        Log.e("team prop location", teamPropLocation + "");
        vision.close();
    }

    public void initVision() {
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, false);
        vision.initCamera(hardwareMap, teamPropDetectionPipeline);
    }

    /**
     * <b>Do preload operation</b> <br>
     * <br>
     * Go to preload position and deposit it
     */

    public void doGroundPreload() {
        Pose2d groundPreloadPosition = robot.drivetrain.getPoseEstimate();

        Log.e("teamPropLocation", teamPropLocation + "");

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition.x += AutoPathConstants.groundPreloadLeftOffset.x;
                groundPreloadPosition.y += AutoPathConstants.groundPreloadLeftOffset.y;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadLeftOffset.heading;
                break;
            case CENTER:
                groundPreloadPosition.x += AutoPathConstants.groundPreloadCenterOffset.x;
                groundPreloadPosition.y += AutoPathConstants.groundPreloadCenterOffset.y;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadCenterOffset.heading;
                break;
            case RIGHT:
                groundPreloadPosition.x += AutoPathConstants.groundPreloadRightOffset.x;
                groundPreloadPosition.y += AutoPathConstants.groundPreloadRightOffset.y;
                groundPreloadPosition.heading += AutoPathConstants.groundPreloadRightOffset.heading;
                break;
        }

        robot.goToPoint(groundPreloadPosition, this);

        start = System.currentTimeMillis();
        if (red) {
            robot.droppers.leftRelease();
        } else {
            robot.droppers.rightRelease();
        }

        while (System.currentTimeMillis() - start < 750) {
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

        robot.deposit.depositAt(15, 0, 2);
        while (!robot.deposit.checkReady()) {
            robot.update();

            if (System.currentTimeMillis() - start > 750) {
                break;
            }
        }
        robot.dunk(1);
    }

    public void park() {
        Pose2d park = AutoPathConstants.parkingLocation.clone();
        park.y *= reflect;

        Pose2d sideDepositLocation = AutoPathConstants.sideDepositLocation.clone();
        sideDepositLocation.y *= reflect;

        robot.goToPoint(sideDepositLocation, this);
        robot.goToPoint(park, this);
    }
}
