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
            //pause(5000);

            doGroundPreload();
            navigateAroundGroundPreload();
            intakeStackInitial();
            doBoardPreload();

            for (int i = 0; i < numCycles; i++) {
                navigateBackToStack();
                intakeStack();
                if (System.currentTimeMillis() - Globals.autoStartTime > 27000) { // if we have less than 5 secs remaining we don't go for the deposit
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
                deposit = new Vector3(5, 0, 9.5);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-45.5, 36, Math.toRadians(50)), this, false, true);
                break;
            case CENTER:
                groundPreloadPosition = new Pose2d(-36.25, 32.5, Math.PI/2);
                boardPreload =          new Pose2d(47, 35.25, Math.PI);
                deposit = new Vector3(5, 0, 9.3);

                robot.goToPoint(groundPreloadPosition, this, false, false);
                break;
            case LEFT:
                groundPreloadPosition = new Pose2d(-36.911, 51, Math.PI/2);
                boardPreload =          new Pose2d(47, 41, Math.PI);
                deposit = new Vector3(5, 0, 9.5);

                robot.goToPoint(groundPreloadPosition, this, false, false);

                robot.goToPoint(new Pose2d(-33.5, 33.5, Math.toRadians(120)), this, false, true);
                break;
        }
        robot.droppers.rightRelease();

        pause(50);
    }

    /**
     * Navigates under stage door
     * If center we route around the ground preload pixel
     */
    public void navigateAroundGroundPreload() {
        switch (teamPropLocation) {
            case RIGHT:
                robot.goToPoint(new Pose2d(-36.5, 34.6, Math.PI/2), this, false, true, 1);
                robot.goToPoint(new Pose2d(-36.5, 11.5, Math.PI/2), this, false, false, 1);
                robot.goToPoint(new Pose2d(-36.5, 11.5, Math.PI), this, false, false, 1);
                break;
            case CENTER:
                robot.goToPoint(new Pose2d(-51.5, 11.5, Math.PI), this, false, false, 1);
                break;
        }
    }

    public void navigateToBoardInital() {
        robot.goToPointWithDepositAndIntake(new Pose2d(15, 10, Math.PI), this, false, false, deposit, 0);
        robot.intake.off();
    }

    Pose2d intakePose = new Pose2d(-59, 13, Math.PI);

    public void navigateBackToStack() {
        robot.intake.on();

        robot.intake.setActuationHeight(pixelIndex); // non destructive
        robot.followSpline(
            new Spline(Globals.ROBOT_POSITION, 3)
                .setReversed(false)
                .addPoint(new Pose2d(29.25, 12, Math.PI), 1.0)
                .addPoint(new Pose2d(-45, 9.5 , Math.PI), 1.0)
                .addPoint(new Pose2d(intakeXDistances[pixelIndex], 7.5, Math.PI), 0.5),
            () -> opModeIsActive() && Globals.NUM_PIXELS != 2 && robot.drivetrain.isBusy(),
            true, false
        );
        robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], 7.5, Math.PI), this, true, false);
    }

    int pixelIndex = 4; // 0 index based
    double[] intakeXDistances = new double[] {-56, -56, -55.45, -55.45, -55.4}; // 1 <-- 5 pixels

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
        robot.goToPoint(new Pose2d(intakeXDistances[pixelIndex], 10.75, Math.PI), this, true, false);

        robot.intake.setActuationHeight(pixelIndex, 0.8);

        pause(750);

        start = System.currentTimeMillis();
        while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 300) {
            robot.intake.setActuationHeight(0, 0.05);
            robot.update();
        }

        Globals.NUM_PIXELS = 2;

        Globals.mergeUltrasonics = false;
        pixelIndex--;
    }

    int cycleNum = 0;
    public void intakeStack() {
        System.out.println(pixelIndex + " is the pixel index");
        Globals.mergeUltrasonics = true;
        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.on();
        robot.intake.setActuationHeight(pixelIndex, 0.5) ;

        start = System.currentTimeMillis();
        while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 300) {
            robot.intake.setActuationHeight(0, 0.07);
            robot.update();
        }

        pixelIndex = Math.max(pixelIndex - 1, 0);

        robot.intake.setActuationHeight(pixelIndex, 0.5) ;

        start = System.currentTimeMillis();
        while (Globals.NUM_PIXELS != 2 && System.currentTimeMillis() - start < 1200) {
            robot.intake.setActuationHeight(0, 0.0725);
            robot.update();
        }

        if (robot.intake.reversed) {
            Log.e("trying to turn intake on", "turning");
            robot.intake.on();
            pause(1000);
        }
        robot.intake.reversed = false;

        Globals.NUM_PIXELS = 2;
        deposit = new Vector3(5, 0, 13.5 + (cycleNum*5));
        Globals.mergeUltrasonics = false;
        cycleNum++;
    }

    /**
     * Do board preload in deposit <br>
     * <br>
     * If it is up it will intake one before it goes to the bord to deposit <br>
     * Ends on the center lane
     */
    public void doBoardPreload() {
        robot.followSplineWithIntakeAndDeposit(
                new Spline(Globals.ROBOT_POSITION, 3)
                        .setReversed(true)
                        .addPoint(new Pose2d(25, 10, Math.PI))
                        .addPoint(new Pose2d(boardPreload.x-7, boardPreload.y-5, boardPreload.heading)),
                deposit,
                20,
                -4,
                1.0,
                false,
                false
        );
        robot.intake.off();

        robot.goToPoint(boardPreload, this::opModeIsActive, true, 0.75, 0.75,1.1, 2.5);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseOne();

//        robot.deposit.release.close();

        deposit = new Vector3(5, 0, 11.5);
        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        pause(300);

        robot.releaseOne();
    }

    public void depositOnBoard() {
        robot.followSplineWithIntakeAndDeposit(
                new Spline(Globals.ROBOT_POSITION, 3)
                        .setReversed(true)
                        .addPoint(new Pose2d(25, 3.5, Math.PI))
                        .addPoint(new Pose2d(40, 28, Math.PI)),
                deposit,
                20,
                -4,
                1.0,
                false,
                false
        );
        robot.intake.off();

        robot.goToPoint(new Pose2d(46.5, 31.8, Math.PI), this::opModeIsActive, true, 0.9, 0.75,2, 2.5);

        robot.depositAt(deposit.z, deposit.x); // sync call to deposit

        robot.releaseOne();
        pause(300);
        robot.releaseOne();
    }

    // Janky method that I don't want to type 3 times
    public void preciseIntake() {
        Log.e("pixelIndex", pixelIndex + "");
        AtomicLong start = new AtomicLong(System.currentTimeMillis());
        double limitYp = intakePose.getY() + 5;
        double limitYn = intakePose.getY() - 5;
        robot.goToPoint(
                new Pose2d(intakeXDistances[pixelIndex], intakePose.getY(), intakePose.getHeading()),
                () -> {
                    int sign = 1;
                    // Hella janky way to insert ourselves into the update() function
                    if ((System.currentTimeMillis() - start.get()) > 500) {
                        //pixelIndex = Math.max(0, index - 1);
                        robot.intake.setActuationAngle(robot.intake.actuation.getTargetAngle() + 0.2, 1);
                        //robot.intake.setActuationHeight(pixelIndex);
                        // Lord forgive me
                        robot.drivetrain.targetPoint.x -= 0.2;
                        start.set(System.currentTimeMillis());
                    }
                    if (robot.drivetrain.targetPoint.y >= limitYp || robot.drivetrain.targetPoint.y <= limitYn) {
                        sign *= -1;
                    }
                    robot.drivetrain.targetPoint.y += 0.2 * sign;
                    return Globals.NUM_PIXELS != 2 && opModeIsActive();
                },
                false, 0.2,
                fx, fy, fh
        );
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