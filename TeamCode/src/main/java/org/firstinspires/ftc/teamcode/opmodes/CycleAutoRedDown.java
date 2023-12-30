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

@Autonomous(name = "RED Cycle Auto Down")
public class CycleAutoRedDown extends LinearOpMode {
    private Vision vision;
    protected TeamPropDetectionPipeline.TeamPropLocation teamPropLocation = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
    protected Robot robot;

    private long start;
    private Vector3 deposit = null;
    private Pose2d boardPreload = null;

    private int numCycles = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            doInitialization();
            waitForStart();

            doGroundPreload();
            navigateAroundGroundPreload();
            intakeStackInitial();
            navigateToBoard();
            doBoardPreload();

            for (int i = 0; i < numCycles; i++) {
                navigateBackToStack();
                intakeStack();
                navigateToBoard();
                depositOnBoard();
            }

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

        robot.drivetrain.setPoseEstimate(new Pose2d(-36, -65, -Math.PI / 2));

        robot.droppers.leftDown();
        robot.droppers.rightRelease();

        robot.hangActuation.up();
        robot.airplane.hold();

        robot.deposit.release.preGrab();

        vision.enableTeamProp();
        vision.disableAprilTag();

        while (opModeInInit() && !isStopRequested()) {
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

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition = new Pose2d(-48, -40.5, -Math.PI/2);
                boardPreload =          new Pose2d(50, -31, Math.PI);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-36, -38.75, -Math.PI/2);
                boardPreload =          new Pose2d(50, -35.5, Math.PI);
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(-36, -49.5, -Math.PI/2);
                boardPreload =          new Pose2d(50, -43, Math.PI);
                break;
        }

        robot.goToPoint(groundPreloadPosition, this, false, false);

        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
            robot.goToPoint(new Pose2d(-34.5, -35, 5*Math.PI/4), this, false, false);
        }

        robot.droppers.leftRelease();

        pause(100);
    }

    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */
    public void navigateAroundGroundPreload() {
        switch (teamPropLocation) {
            case LEFT:
                robot.goToPoint(new Pose2d(-57, -39, -Math.PI/2), this, false, false);
                robot.goToPoint(new Pose2d(-57, -12, -Math.PI/2), this, false, false);
                robot.goToPoint(new Pose2d(-57, -12, Math.PI), this, false, false);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-52, -12, Math.PI), this, false, false);
                break;
            case RIGHT:
                robot.goToPoint(new Pose2d(-37.5, -12, Math.PI), this, false, false);
                break;
        }
    }

    public void navigateToBoard() {
        deposit = new Vector3(5, 0, 6);
        robot.intake.reverse();
        robot.goToPointWithDeposit(new Pose2d(28, -12, Math.PI), this, false, false, deposit, 0);
        robot.intake.off();
    }

    public void navigateBackToStack() {
        robot.goToPoint(new Pose2d(28, -12, Math.PI), this, false, false);
        robot.intake.on();
        robot.intake.setActuationHeight(pixelIndex);
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        robot.goToPoint(boardPreload, this, true, true);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseTwo();
    }

    public void depositOnBoard() {
        robot.goToPoint(new Pose2d(50, -31, Math.PI), this, true, true);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseTwo();
    }

    Pose2d intakePose = new Pose2d(-58, -12, Math.PI);

    int pixelIndex = 4; // 0 index based
    public void intakeStackInitial() {
        robot.goToPoint(intakePose, this, true, true);
        robot.intake.setActuationHeight(pixelIndex);
        robot.intake.on();
        pixelIndex--;
        pause(750);
        Globals.NUM_PIXELS = 2;
    }

    public void intakeStack() {
        robot.goToPoint(intakePose, this, true, true);
        pause(750);
        pixelIndex--;
        robot.intake.setActuationHeight(pixelIndex);
        pause(750);
        pixelIndex--;
        deposit = new Vector3(5, 0, 12);
        Globals.NUM_PIXELS = 2;
    }

    /**
     * Assumes that it is in the parking line row
     */
    public void park() {
        robot.goToPoint(new Pose2d(42, -12, Math.PI), this, false, false); // intermediate parking
        robot.goToPoint(new Pose2d(58, -12, Math.PI), this, false, true); // parking

        Globals.AUTO_ENDING_POSE = robot.drivetrain.getPoseEstimate();
    }

    public void pause (double milliseconds) {
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds && opModeIsActive()) {
            robot.update();
        }
    }
}
