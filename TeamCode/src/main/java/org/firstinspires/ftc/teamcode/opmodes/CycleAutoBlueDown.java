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
                if (System.currentTimeMillis() - Globals.autoStartTime > 24000) { // if we have less than 2 secs remaining we don't go for the deposit
                    break;
                }
                navigateBackToStack();
                intakeStack();
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

    double intakeYDistance = 13.25;

    public void doGroundPreload() {
        Pose2d groundPreloadPosition;

        Log.e("teamPropLocation", teamPropLocation + "");

        switch (teamPropLocation) {
            case RIGHT:
                groundPreloadPosition = new Pose2d(-46, 42, Math.PI/2);
                boardPreload =          new Pose2d(47.45, 29, Math.PI);
                deposit = new Vector3(5, 0, 9.5);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-48, 37.5, Math.toRadians(55)), this, false, true);

                intakeXDistances[4] = -58.25;
                intakeYDistance = 13;
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-39, 37.5, Math.PI/2);
                boardPreload =          new Pose2d(47.25, 34.6, Math.PI);
                deposit = new Vector3(5, 0, 9.15);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                intakeXDistances[4] = -58.25;
                intakeYDistance = 12.25;
                break;
            case LEFT:
                groundPreloadPosition = new Pose2d(-36.911, 51, Math.PI/2);
                boardPreload =          new Pose2d(47, 41.5, Math.PI);
                deposit = new Vector3(5, 0, 9.5);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-36, 33.5, Math.toRadians(120)), this, false, true);

                intakeXDistances[4] = -58.25;
                intakeYDistance = 12.5;
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
                robot.goToPoint(new Pose2d(-43, 40,  Math.toRadians(40)), this, false, false, 1);
                robot.goToPoint(new Pose2d(-61, 40, Math.PI/2), this, false, false, 1);
                robot.goToPoint(new Pose2d(-61, 11.5, Math.PI/2), this, false, false, 1);
                robot.goToPoint(new Pose2d(-48, 11.5, Math.PI), this, false, false, 1);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-51.5, 11.5, Math.PI), this, false, false, 1);
                break;
        }
    }



    public void navigateBackToStack() {
        robot.intake.setActuationHeight(pixelIndex); // non destructive

        robot.goToPoint(new Pose2d(29.25, 10, Math.PI), this, false, false);
        robot.intake.on();
        robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], intakeYDistance, Math.PI), this::opModeIsActive, true, 1.0, 0.75, 1, Math.toDegrees(1.25));
    }

    int pixelIndex = 4; // 0 index based
    double[] intakeXDistances = new double[] {-56.7, -56.7, -57, -56.8, -57}; // 1 <-- 5 pixels

    public void intakeStackInitial() {
        Globals.mergeUltrasonics = true;
        robot.intake.on();
        robot.intake.setActuationHeightSlightlyAbove(pixelIndex);

        robot.followSpline(
            new Spline(Globals.ROBOT_POSITION, 3)
                .setReversed(false)
                .addPoint(new Pose2d(-48.5, 11.5, Math.PI))
                .addPoint(new Pose2d(intakeXDistances[pixelIndex], 11.5, Math.PI)),
            this::opModeIsActive
        );
        robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], 11.5, Math.PI), this, true, false);

        robot.intake.setActuationHeight(pixelIndex, 1.0);

        pause(350);

        start = System.currentTimeMillis();
        while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 400) {
            robot.intake.setActuationHeight(0, 0.0725);
            robot.update();
        }

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 75) {
            robot.intake.reverse();
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
        if (pixelIndex <= 1) {  // final cycle
            robot.intake.setActuationHeight(0, 1.0);
        } else {
            robot.intake.setActuationHeight(pixelIndex, 1.0) ;
        }

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 400) {
            robot.intake.setActuationHeight(0, 0.9);
            robot.update();
        }

        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.setActuationHeight(pixelIndex, 1.0);

        start = System.currentTimeMillis();
        if (pixelIndex < 1) {
            robot.intake.setActuationHeight(0, 1.0);
            if (Globals.NUM_PIXELS != 2) {
                if (teamPropLocation != TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
                    robot.goToPoint(new Pose2d(-57, intakeYDistance + 7, Math.PI), this::opModeIsActive, true, 1.0, 2.5, 1, Math.toDegrees(2.5));
                } else {
                    robot.goToPoint(new Pose2d(-57, intakeYDistance + 5, Math.PI), this::opModeIsActive, true, 1.0, 2.5, 1, Math.toDegrees(2.5));
                    robot.goToPoint(new Pose2d(-57, intakeYDistance + 12.75, Math.PI), this::opModeIsActive, true, 1.0, 2.5, 1, Math.toDegrees(2.5));
                }
                robot.goToPoint(new Pose2d(-57, intakeYDistance, Math.PI), this, false, false, 1.0);
            }
        } else {
//            while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 1000) {
//                robot.intake.setActuationHeight(0, 0.0725);
//                robot.update();
//            }
            robot.intake.setActuationHeight(0, 0.0725);
            robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], intakeYDistance-3.5, Math.PI), this, false, false, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], intakeYDistance+3.5, Math.PI), this, false, false, 1.0);
            robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], intakeYDistance, Math.PI), this, false, false, 1.0);
        }

        if (robot.intake.reversed) {
            Log.e("trying to turn intake on", "turning");
            robot.intake.on();
            pause(100);
        }
        robot.intake.reversed = false;

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 50) {
            robot.intake.reverse();
            robot.update();
        }

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
                        .addPoint(new Pose2d(36, 9, Math.PI))
                        .addPoint(new Pose2d(36, 15, (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT) ? -Math.toRadians(130) : -Math.toRadians(140))), //if the teamprop is to the right change the angle to get angle of board
                0,
                -15,
                1.0,
                false,
                true
        );
        robot.checkForPartner(this, 500);
        robot.deposit.depositAt(new Vector3(5,0,10.25)); // async call to deposit

        robot.goToPoint(new Pose2d(boardPreload.x-8, boardPreload.y, Math.PI), this, false, false);
        robot.intake.off();
        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.goToPointWithLimitSwitch(boardPreload, this, true, 1.0);

        robot.releaseOne();

        deposit = new Vector3(5, 0, deposit.z+5.5);
        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        pause(350);

        robot.releaseOne();
    }

    int cycleNum = 0;
    double[] depositHeights = new double[] {14, 18};

    public void depositOnBoard() {
        deposit.z = depositHeights[cycleNum];
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT) {
            deposit.z = deposit.z + 4;
        }

        robot.followSplineWithIntakeAndDeposit(
                new Spline(new Pose2d(Globals.ROBOT_POSITION.x, 8, Globals.ROBOT_POSITION.heading), 5)
                        .setReversed(true)
                        .addPoint(new Pose2d(30, 8, Math.PI))
                        .addPoint(new Pose2d(38, 24, Math.PI)),
                deposit,
                0,
                -15,
                1.0,
                false,
                false
        );

//        robot.goToPointWithDepositAndIntake(new Pose2d(30, 8, Math.PI), this, false, false, deposit,0, -40);
//        robot.goToPoint(new Pose2d(30, 24, Math.PI), this, false, false);

        robot.intake.off();
        robot.deposit.depositAt(deposit); // async call

        robot.goToPointWithLimitSwitch(new Pose2d(47.25, 28.5, Math.PI), this, true, 1.0);

        robot.releaseTwo();
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