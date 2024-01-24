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

            pause(1000);
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

        robot.drivetrain.setPoseEstimate(new Pose2d(-36.911, -61, -Math.PI / 2));

        robot.droppers.leftDown();
        robot.droppers.rightRelease();

        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        while (opModeInInit() && !isStopRequested()) {
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
        Pose2d groundPreloadPosition;

        Log.e("teamPropLocation", teamPropLocation + "");

        switch (teamPropLocation) {
            case LEFT:
                groundPreloadPosition = new Pose2d(-35.411, -42.5, -Math.PI/2);
                boardPreload =          new Pose2d(50, -30.25, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-43.4, -34.6, -Math.toRadians(50)), this, false, true);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-36.25, -37.25, -Math.PI/2);
                boardPreload =          new Pose2d(50, -36.25, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(-35.411, -49.5, -Math.PI/2);
                boardPreload =          new Pose2d(50, -41.41, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-32, -33.5, -Math.toRadians(140)), this, false, true);
                break;
        }
        robot.droppers.leftRelease();

        pause(150);
    }
    Pose2d rightInFrontOfStackPose = new Pose2d(-50, -12, Math.PI);
    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */
    public void navigateAroundGroundPreload() {
        switch (teamPropLocation) {
            case LEFT:
                robot.goToPoint(new Pose2d(-34.0, -34.6, -Math.PI/2), this, false, true);
                robot.goToPoint(new Pose2d(-34.0, -11.5, -Math.PI/2), this, false, false);
                robot.goToPoint(new Pose2d(-34.0, -11.5, Math.PI), this, false, false);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-51.5, -11.5, Math.PI), this, false, false);
                break;
            case RIGHT:
                robot.goToPoint(new Pose2d(-37, -11.5, Math.PI), this, false, false);
                break;
        }

        robot.intake.setActuationHeight(pixelIndex);
        robot.splineToPoint(rightInFrontOfStackPose, this, false, true, true);
    }

    public void navigateToBoard() {
        robot.goToPointWithDepositAndIntake(new Pose2d(32, -10, Math.PI), this, false, false, deposit, 0);
        robot.intake.off();
    }

    public void navigateBackToStack() {
        robot.splineToPoint(new Pose2d(27.41, -10, Math.toRadians(150)), this, false, false, false);
        robot.intake.on();
        robot.intake.setActuationHeight(pixelIndex);
        robot.splineToPoint(rightInFrontOfStackPose, this, false, true, false);
    }

    int pixelIndex = 4; // 0 index based
    double[] actuationDistances = new double[] {12.75, 12.75, 12.75, 12.75, 12.75}; // 1 <-- 5 pixels

    Pose2d intakePose = new Pose2d(-58, -12, Math.PI); // Eric: Prev -55

    public void intakeStackInitial() {
        Globals.mergeUltrasonics = true;
        robot.intake.on();
        robot.splineToPoint(intakePose, this, true, true, true);
        pixelIndex--;
        pause(300);
        Globals.NUM_PIXELS = 2;
        deposit = new Vector3(5, 0, 9.5);
        Globals.mergeUltrasonics = false;
    }

    public void intakeStack() {
        Globals.mergeUltrasonics = true;
        deposit = new Vector3(5, 0, 18);
        robot.splineToPoint(intakePose, this, true, true, true);
        pause(300);
        pixelIndex--;
        robot.intake.setActuationHeight(pixelIndex);
        pause(300);
        pixelIndex--;
        Globals.NUM_PIXELS = 2;
        deposit = new Vector3(5, 0, 9+(4-pixelIndex));
        Globals.mergeUltrasonics = false;
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        robot.splineToPoint(boardPreload, this, true, true, true);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseOne();

        robot.deposit.release.close();

        deposit = new Vector3(5, 0, 11.5);
        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        pause(300);

        robot.releaseOne();
    }

    public void depositOnBoard() {
        robot.splineToPoint(new Pose2d(49.75, -27.75, Math.PI), this, false, true, true);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseOne();
        pause(300);
        robot.releaseOne();
    }

    /**
     * Assumes that it is in the parking line row
     */
    public void park() {
        robot.splineToPoint(new Pose2d(42, -12, Math.PI), this, false, false, true); // intermediate parking
        robot.splineToPoint(new Pose2d(58, -12, Math.PI), this, false, true, true); // parking

        robot.drivetrain.forceStopAllMotors();
        Globals.AUTO_ENDING_POSE = robot.drivetrain.getPoseEstimate();
    }

    public void pause (double milliseconds) {
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds && opModeIsActive()) {
            robot.update();
        }
    }
}
