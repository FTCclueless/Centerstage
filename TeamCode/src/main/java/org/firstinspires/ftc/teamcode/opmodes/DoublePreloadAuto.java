package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

/**
 * Auto that just parks
 *
 * TODO: There is no blue side support
 */

@Autonomous(name = "Double Preload Auto")
public class DoublePreloadAuto extends LinearOpMode {
    protected Robot robot;
    protected TeamPropDetectionPipeline.TeamPropLocation teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
    protected boolean up = true;
    protected boolean red = true;
    protected int reflect = 1;

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
        robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.AUTO;
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

        Vision vision = new Vision();
        TeamPropDetectionPipeline teamPropDetectionPipeline;

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, true);
        vision.initCamera(hardwareMap, teamPropDetectionPipeline);

        Pose2d targetBoard = AutoPathConstants.targetBoard.clone();
        targetBoard.y *= reflect;
        robot.deposit.setTargetBoard(targetBoard);

        // TODO: Add disable vision flag in case of complications :)
        while (opModeInInit()) {
            teamPropLocation = teamPropDetectionPipeline.getTeamPropLocation();
            teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);
        }
        Log.e("team prop location", teamPropLocation + "");
    }

    /**
     * <b>Do preload operation</b> <br>
     * <br>
     * Go to preload position and deposit it
     */
    public void doGroundPreload() {
        Pose2d groundPreloadPosition = robot.drivetrain.getPoseEstimate();

        Log.e("teamPropLocation", teamPropLocation + "");
        Log.e("groundPreloadPosition.x", groundPreloadPosition.x + "");
        Log.e("groundPreloadPosition.y", groundPreloadPosition.y + "");
        Log.e("groundPreloadPosition.heading (deg)", Math.toDegrees(groundPreloadPosition.heading) + "");

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

        Log.e("groundPreloadPosition.x", groundPreloadPosition.x + "");
        Log.e("groundPreloadPosition.y", groundPreloadPosition.y + "");
        Log.e("groundPreloadPosition.heading (deg)", Math.toDegrees(groundPreloadPosition.heading) + "");

        robot.goToPoint(groundPreloadPosition, this);
        // TODO depo goofy
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

        robot.deposit.depositAt(20, 0, 2);
        while (!robot.deposit.checkReady()) {
            robot.update();
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
