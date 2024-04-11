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

@Autonomous(name = "RED Double Preload Auto Up")
public class DoublePreloadAutoRedUp extends LinearOpMode {
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

        vision = new Vision(hardwareMap, telemetry, true, true, true);
        robot = new Robot(hardwareMap, vision);

        robot.droppers.rightDown();
        robot.droppers.leftRelease();

        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        robot.hang.quickTurnOnOff();

        while (opModeInInit() && !isStopRequested()) {
            teamPropLocation = vision.teamPropDetectionPipeline.getTeamPropLocation();
            vision.teamPropDetectionPipeline.sendTeamPropTelemetry(telemetry);
            robot.deposit.release.preGrab();
            robot.drivetrain.setPoseEstimate(new Pose2d(11.804, -59.75, -Math.PI / 2)); // TODO: Change initial starting position to be 3 inches + in the y

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
                groundPreloadPosition = new Pose2d(11.8, -43.5, -Math.PI/2);
                boardPreload =          new Pose2d(49, -29.75, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                robot.goToPoint(new Pose2d(9, -35, -Math.toRadians(70)), this, false, false);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(24, -32.5, Math.PI), this, false, false);

                deposit = new Vector3(5, 0, 9.5);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(15, -33.5, -Math.PI/2);
                boardPreload =          new Pose2d(49, -37.25, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, true);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(15, -38.5, -Math.PI/2), this, false, false);
                robot.goToPoint(new Pose2d(15, -38.5, Math.PI), this, false, false);

                deposit = new Vector3(5, 0, 8.5);
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(20, -46.5, -Math.PI/2);
                boardPreload =          new Pose2d(49, -43.25, Math.PI);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                robot.goToPoint(new Pose2d(27, -37.5, -Math.toRadians(110)), this, false, true);

                releaseAndTriggerDeposit();

                robot.goToPoint(new Pose2d(21, -37.5, Math.PI), this, false, true);

                deposit = new Vector3(5, 0, 9.5);
                break;
        }
    }

    public void releaseAndTriggerDeposit() {
        robot.droppers.rightRelease();
        pause(150);

        robot.deposit.depositAt(deposit); // async call to deposit
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        robot.goToPointWithLimitSwitch(boardPreload, this, true, 1.0);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseTwo();
    }

    /**
     * Assumes that it is in the parking line row
     */
    public void park() {
        robot.goToPoint(new Pose2d(42, -60, Math.PI), this); // intermediate parking
        robot.goToPoint(new Pose2d(58, -60, Math.PI), this, false, true); // parking

        Globals.AUTO_ENDING_POSE = robot.drivetrain.getPoseEstimate();

        pause (2000);
    }

    public void pause (double milliseconds) {
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < milliseconds && opModeIsActive()) {
            robot.update();
        }
    }
}
