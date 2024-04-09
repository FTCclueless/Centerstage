package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

import java.util.concurrent.atomic.AtomicLong;

@Autonomous(name = "BLUE Cycle Auto Down")
public class CycleAutoBlueDown extends LinearOpMode {
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
            navigateAroundGroundPreload();
            intakeStackInitial();
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

        vision = new Vision(hardwareMap, telemetry, false, true, true);
        robot = new Robot(hardwareMap, vision);

        robot.droppers.rightDown();
        robot.droppers.leftRelease();

        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        robot.hang.quickTurnOnOff();

        while (opModeInInit() && !isStopRequested()) {
            robot.drivetrain.setPoseEstimate(new Pose2d(-36.911, 60.75, Math.PI / 2));

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
            case RIGHT:
                groundPreloadPosition = new Pose2d(-36.25, 42.5, Math.PI/2);
                boardPreload =          new Pose2d(47, 28, Math.PI);
                deposit = new Vector3(5, 0, 10);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-47, 37.5, Math.toRadians(55)), this, false, true);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-39, 40, Math.PI/2);
                boardPreload =          new Pose2d(47, 34.35, Math.PI);
                deposit = new Vector3(5, 0, 9.1);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                break;
            case LEFT:
                groundPreloadPosition = new Pose2d(-36.911, 51, Math.PI/2);
                boardPreload =          new Pose2d(47, 42.15, Math.PI);
                deposit = new Vector3(5, 0, 10);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-34.5, 33.5, Math.toRadians(120)), this, false, true);
                break;
        }
        robot.droppers.rightRelease();

        pause(150);
    }

    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */
    public void navigateAroundGroundPreload() {
        switch (teamPropLocation) {
            case RIGHT:
                robot.goToPoint(new Pose2d(-37.25, 34.6, Math.PI/2), this, false, true, 1);
                robot.goToPoint(new Pose2d(-37.25, 11.5, Math.PI/2), this, false, false, 1);
                robot.goToPoint(new Pose2d(-37.25, 11.5, Math.PI), this, false, false, 1);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-51.5, 11.5, Math.PI), this, false, false, 1);
                break;
        }
    }

    double intakeYDistance = 9.5;

    public void navigateBackToStack() {
        robot.intake.on();

        robot.intake.setActuationHeight(pixelIndex); // non destructive
        robot.followSpline(
            new Spline(Globals.ROBOT_POSITION, 3)
                .setReversed(false)
                .addPoint(new Pose2d(29.25, 8.5, Math.PI))
                .addPoint(new Pose2d(intakeXDistances[pixelIndex], intakeYDistance, Math.PI), 1.0),
            () -> opModeIsActive() && Globals.NUM_PIXELS != 2 && robot.drivetrain.isBusy()
        );
    }

    int pixelIndex = 4; // 0 index based
    double[] intakeXDistances = new double[] {-55.4, -55.4, -55.2, -55.2, -55.65}; // 1 <-- 5 pixels

    public void intakeStackInitial() {
        Globals.mergeUltrasonics = true;
        robot.intake.on();
        robot.intake.setActuationHeightSlightlyAbove(pixelIndex);

        robot.followSpline(
            new Spline(Globals.ROBOT_POSITION, 3)
                .setReversed(false)
                .addPoint(new Pose2d(-48.5, 12, Math.PI))
                .addPoint(new Pose2d(intakeXDistances[pixelIndex], 12, Math.PI)),
            this::opModeIsActive
        );
        robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], 12, Math.PI), this, true, false);

        robot.intake.setActuationHeight(pixelIndex, 1.0);

        pause(300);

        start = System.currentTimeMillis();
        while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 750) {
            robot.intake.setActuationHeight(0, 0.04);
            robot.update();
        }

        Globals.NUM_PIXELS = 2;

        Globals.mergeUltrasonics = false;
        pixelIndex--;
    }

    public void intakeStack() {
        Globals.mergeUltrasonics = true;
        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.on();
        robot.intake.setActuationHeight(pixelIndex, 0.5) ;

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 650) {
            robot.intake.setActuationHeight(0, 0.05);
            robot.update();
        }

        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.setActuationHeight(pixelIndex, 0.5) ;

        start = System.currentTimeMillis();
        if (pixelIndex < 1 && Globals.NUM_PIXELS != 2) {
            robot.intake.setActuationHeight(0, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[1], intakeYDistance-6, Math.PI), this, false, false, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[1], intakeYDistance+17, Math.PI), this, false, false, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[1], intakeYDistance, Math.PI), this, false, false, 1.0);
        } else {
            while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 1200) {
                robot.intake.setActuationHeight(0, 0.0725);
                robot.update();
            }
        }

        if (robot.intake.reversed) {
            Log.e("trying to turn intake on", "turning");
            robot.intake.on();
            pause(1000);
        }
        robot.intake.reversed = false;

        Globals.NUM_PIXELS = 2;
        Globals.mergeUltrasonics = false;
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        robot.followSplineWithIntake(
                new Spline(Globals.ROBOT_POSITION, 3)
                        .setReversed(true)
                        .addPoint(new Pose2d(31, 10, Math.PI))
                        .addPoint(new Pose2d(31, 15, (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT) ? -Math.toRadians(130) : -Math.toRadians(140))), //if the teamprop is to the right change the angle to get angle of board
                0,
                -40,
                1.0,
                false,
                true
        );
        robot.checkForPartner(this,750);
        robot.deposit.depositAt(new Vector3(5,0,10.25)); // async call to deposit

        robot.goToPoint(new Pose2d(boardPreload.x-6, boardPreload.y, Math.PI), this, false, false);
        robot.intake.off();
        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.goToPointWithLimitSwitch(boardPreload, this, true, 1.0);

        robot.releaseOne();

        deposit = new Vector3(5, 0, deposit.z+6);
        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        pause(350);

        robot.releaseOne();
    }

    int cycleNum = 0;
    double[] depositHeights = new double[] {15, 18.5};

    public void depositOnBoard() {
        deposit.z = depositHeights[cycleNum];
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
            deposit.z = deposit.z + 5;
        }

        robot.followSplineWithIntakeAndDeposit(
                new Spline(Globals.ROBOT_POSITION, 3)
                        .setReversed(true)
                        .addPoint(new Pose2d(28, 3.5, Math.PI))
                        .addPoint(new Pose2d(43, 25, Math.PI)),
                deposit,
                0,
                -4,
                1.0,
                false,
                false
        );
        robot.intake.off();
        robot.deposit.depositAt(deposit); // async call

        robot.goToPointWithLimitSwitch(new Pose2d(46.5, 29, Math.PI), this, true, 1.0);

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