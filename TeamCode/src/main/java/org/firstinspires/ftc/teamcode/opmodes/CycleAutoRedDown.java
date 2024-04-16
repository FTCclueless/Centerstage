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

@Autonomous(name = "RED Cycle Auto Down")
public class CycleAutoRedDown extends LinearOpMode {
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
        Globals.isRed = true;
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

        vision = new Vision(hardwareMap, telemetry, true, true, true);
        robot = new Robot(hardwareMap, vision);

        robot.droppers.leftDown();
        robot.droppers.rightRelease();

        robot.airplane.hold();

        vision.enableTeamProp();
        vision.disableAprilTag();

        robot.hang.quickTurnOnOff();

        while (opModeInInit() && !isStopRequested()) {
            robot.drivetrain.setPoseEstimate(new Pose2d(-36.911, -60.75, -Math.PI / 2));

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
                groundPreloadPosition = new Pose2d(-45, -42.5, -Math.PI/2);
                boardPreload =          new Pose2d(46.25, -29.5, Math.PI);
                deposit = new Vector3(5, 0, 9.5);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-48.25, -37.5, -Math.toRadians(60)), this, false, true);

                intakeYDistance = -13.5;
                initialIntakeStackDistance = -53.35;
                initialIntakeYDistance = -13;
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-36.25, -35.5, -Math.PI/2);
                boardPreload =          new Pose2d(46.25, -36, Math.PI);
                deposit = new Vector3(5, 0, 9);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                intakeYDistance = -13.5;
                initialIntakeStackDistance = -53.35;
                initialIntakeYDistance = -12;
                break;
            case RIGHT:
                groundPreloadPosition = new Pose2d(-36.911, -51, -Math.PI/2);
                boardPreload =          new Pose2d(46.25, -42, Math.PI);
                deposit = new Vector3(5, 0, 9.5);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-34.25, -35, -Math.toRadians(115)), this, false, true);

                intakeYDistance = -13.5;
                initialIntakeStackDistance = -53.35;
                initialIntakeYDistance = -12;
                break;
        }
        robot.droppers.leftRelease();

        pause(150);
    }

    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */
    public void navigateAroundGroundPreload() {
        switch (teamPropLocation) {
            case LEFT:
                robot.goToPoint(new Pose2d(-43, -40, -Math.toRadians(40)), this, false, false, 1);
                robot.goToPoint(new Pose2d(-59, -40, -Math.toRadians(90)), this, false, false, 1);
                robot.goToPoint(new Pose2d(-59, -11.5, -Math.toRadians(90)), this, false, false, 1);
                robot.goToPoint(new Pose2d(-45, -11.5, Math.toRadians(180)), this, false, false, 1);

//                robot.goToPoint(new Pose2d(-41.5, -40.5, -Math.toRadians(60)), this, false, false, 1);
//                robot.goToPoint(new Pose2d(-41.5, -40, -Math.PI/2), this, false, false, 1);
//                robot.goToPoint(new Pose2d(-36.5, -40, -Math.PI/2), this, false, false, 1);
//                robot.goToPoint(new Pose2d(-36.5, -11.5, -Math.PI/2), this, false, false, 1);
//                robot.goToPoint(new Pose2d(-36.5, -11.5, Math.PI), this, false, false, 1);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-51.5, -11.5, Math.PI), this, false, false, 1);
                break;
        }
    }

    double intakeYDistance = -13.7;

    public void navigateBackToStack() {
        robot.intake.setActuationHeight(pixelIndex); // non destructive

        robot.goToPoint(new Pose2d(29.25, -12, Math.PI), this, false, false);
        robot.intake.on();
        robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], intakeYDistance, Math.PI), this::opModeIsActive, true,1.0, 1, 1, Math.toDegrees(1.25));
    }

    int pixelIndex = 4; // 0 index based
    double[] intakeXDistances = new double[] {-54.5, -54.5, -54.5, -54.5, -54.5}; // 1 <-- 5 pixels
    double initialIntakeStackDistance = -55;
    double initialIntakeYDistance = -12;

    public void intakeStackInitial() {
        Globals.mergeUltrasonics = true;
        robot.intake.on();
        robot.intake.setActuationHeightSlightlyAbove(pixelIndex);

        robot.followSpline(
            new Spline(Globals.ROBOT_POSITION, 3)
                .setReversed(false)
                .addPoint(new Pose2d(-48.5, -12, Math.PI))
                .addPoint(new Pose2d(initialIntakeStackDistance, initialIntakeYDistance, Math.PI)),
            this::opModeIsActive
        );

        robot.goToPoint(new Pose2d(initialIntakeStackDistance, initialIntakeYDistance, Math.PI), this, true, false);

        robot.intake.setActuationHeight(pixelIndex, 1.0);

        pause(500);

        start = System.currentTimeMillis();
        while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 500) {
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
            if (Globals.NUM_PIXELS != 2  && teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT) {
                robot.goToPoint(new Pose2d(-55, intakeYDistance+4, Math.PI), this, false, false, 1.0);
                robot.goToPoint(new Pose2d(-55, intakeYDistance-10, Math.PI), this, false, false, 1.0);
            } else {
                robot.goToPoint(new Pose2d(-55, intakeYDistance-17, Math.PI), this, false, false, 1.0);
            }
            robot.goToPoint(new Pose2d(-52.5, intakeYDistance, Math.PI), this, false, false, 1.0);
        } else {
            while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 1000) {
                robot.intake.setActuationHeight(0, 0.0725);
                robot.update();
            }
        }

        if (robot.intake.reversed) {
            Log.e("trying to turn intake on", "turning");
            robot.intake.on();
            pause(250);
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
                        .addPoint(new Pose2d(36, -9, Math.PI))
                        .addPoint(new Pose2d(36, -15, (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.RIGHT) ? Math.toRadians(130) : Math.toRadians(140))), //if the teamprop is to the right change the angle to get angle of board
                0,
                -20,
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
        if (teamPropLocation == TeamPropDetectionPipeline.TeamPropLocation.LEFT) {
            deposit.z = deposit.z + 4;
        }

        robot.followSplineWithIntakeAndDeposit(
                new Spline(Globals.ROBOT_POSITION, 5)
                        .setReversed(true)
                        .addPoint(new Pose2d(30, -9.25, Math.PI))
                        .addPoint(new Pose2d(38, -31.5, Math.PI)),
                deposit,
                0,
                -5,
                1.0,
                false,
                false
        );
        robot.intake.off();
        robot.deposit.depositAt(deposit); // async call

        robot.goToPointWithLimitSwitch(new Pose2d(46.25,-31.5, Math.PI), this, true, 0.25);

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