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

@Autonomous(name = "Park Auto")
public class ParkAuto extends LinearOpMode {
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
            robot.drivetrain.setPoseEstimate(APC.startUp);
        } else {
            robot.drivetrain.setPoseEstimate(APC.startDown);
        }

        Vision vision = new Vision();
        TeamPropDetectionPipeline teamPropDetectionPipeline;

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, true);
        vision.initCamera(hardwareMap, teamPropDetectionPipeline);

        // TODO: Add disable vision flag in case of complications :)
        teamPropLocation = teamPropDetectionPipeline.getTeamPropLocation();
    }

    /**
     * <b>Do preload operation</b> <br>
     * <br>
     * Go to preload position and deposit it
     */
    public void doGroundPreload() {
        Pose2d parkPosition = robot.drivetrain.getPoseEstimate();

        switch (teamPropLocation) {
            case LEFT:
                parkPosition.x += APC.groundPreloadTopOffset.x;
                parkPosition.y += APC.groundPreloadTopOffset.y;
                parkPosition.heading += APC.groundPreloadTopOffset.heading;
                break;
            case CENTER:
                parkPosition.x += APC.groundPreloadCenterOffset.x;
                parkPosition.y += APC.groundPreloadCenterOffset.y;
                parkPosition.heading += APC.groundPreloadCenterOffset.heading;
                break;
            case RIGHT:
                parkPosition.x += APC.groundPreloadBottomOffset.x;
                parkPosition.y += APC.groundPreloadBottomOffset.y;
                parkPosition.heading += APC.groundPreloadBottomOffset.heading;
                break;
            case NONE:
            default:
                Log.e("ParkAuto", "Error! No team prop location!");
        }

        robot.goToPoint(parkPosition, this);
        // TODO depo goofy
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        Pose2d boardPreload = APC.boardPreload.clone();
        boardPreload.y *= reflect;

        Pose2d initialIntake = APC.initialIntake.clone();
        initialIntake.y *= reflect;

        // Up behavior is to instantly deposit preload on board
        if (!up)
            robot.goToPoint(initialIntake, this);
            // TOOD: Intake

        robot.goToPoint(boardPreload, this);
        robot.goToPoint(boardPreload.x, APC.depositLocation.y, APC.depositLocation.heading, this);
        // Deposit
    }

    public void park() {
        Pose2d park = APC.parkingLocation.clone();
        park.y *= reflect;

        robot.goToPoint(park, this);
    }
}
