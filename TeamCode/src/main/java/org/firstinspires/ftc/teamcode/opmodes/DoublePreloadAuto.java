package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

@Config
@TeleOp
public class DoublePreloadAuto extends LinearOpMode {
    private boolean up = false; // Is on top side of field
    private boolean blue = false;

    REDGroundPreloadAuto.PreloadGlobal preloadGlobal = REDGroundPreloadAuto.PreloadGlobal.CENTER;

    private TeamPropDetectionPipeline.TEAM_PROP_LOCATION team_prop_location = TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.AUTO;

        Vision vision = new Vision();
        TeamPropDetectionPipeline teamPropDetectionPipeline;

        // TODO: add initalization sequence
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, true);
        vision.initCamera(hardwareMap, teamPropDetectionPipeline);

        while (opModeInInit()) {
            team_prop_location = teamPropDetectionPipeline.getTeamPropLocation();

            telemetry.addData("leftAvg", teamPropDetectionPipeline.leftAvg);
            telemetry.addData("centerAvg", teamPropDetectionPipeline.centerAvg);
            telemetry.addData("rightAvg", teamPropDetectionPipeline.rightAvg);
            telemetry.addData("propLocation: ", team_prop_location);
            telemetry.update();
        }

        int reflect = blue ? 1 : -1; // Reflect for blue side

        if (up) {
            robot.drivetrain.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(90 * reflect)));
        } else {
            robot.drivetrain.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(90 * reflect))); // up and down are mixed together
        }

        waitForStart();

        // Wubba lubba dub dub
        Pose2d pose = robot.drivetrain.getPoseEstimate();
        if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER) {
            preloadGlobal = REDGroundPreloadAuto.PreloadGlobal.CENTER;
        } else if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.LEFT && blue || team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.RIGHT && !blue) {
            preloadGlobal = REDGroundPreloadAuto.PreloadGlobal.TOP;
        } else {
            preloadGlobal = REDGroundPreloadAuto.PreloadGlobal.BOTTOM;
        }

        long start = 0;


        robot.goToPoint(new Pose2d(12, 60 * reflect, 0), this);

        if (up) {
            switch (preloadGlobal) {
                case TOP:
                    robot.goToPoint(new Pose2d(48, 42 * reflect, 0), this);
                    robot.depositAt(8,6 * reflect);
                    robot.deposit.dunk(1);
                    while (robot.deposit.state != Deposit.State.WAIT) {
                        robot.update();
                    }

                    robot.goToPoint(new Pose2d(36, 36 * reflect, Math.PI), this);

                    break;

                case CENTER:
                    robot.goToPoint(new Pose2d(48, 36 * reflect, 0), this);
                    robot.depositAt(8,0);
                    robot.deposit.dunk(1);
                    while (robot.deposit.state != Deposit.State.WAIT) {
                        robot.update();
                    }

                    robot.goToPoint(new Pose2d(30, 24 * reflect, Math.PI), this);

                    break;
                case BOTTOM:
                    robot.goToPoint(new Pose2d(48, 30*reflect, 0), this);
                    robot.depositAt(8, 6);
                    robot.deposit.dunk(1);
                    while (robot.deposit.state != Deposit.State.WAIT) {
                        robot.update();
                    }

                    robot.goToPoint(new Pose2d(12, 36 * reflect, Math.PI), this);
            }

            start = System.currentTimeMillis();
            robot.intake.actuationDown();
            while(System.currentTimeMillis() - start <= 500) {
                robot.update();
            }
            start = System.currentTimeMillis();
            while(System.currentTimeMillis() - start <= 2000) {
                robot.intake.reverse();
                robot.update();
            }
            robot.intake.off();

            robot.goToPoint(new Pose2d(36, 12 * reflect, 0), this);
            robot.goToPoint(new Pose2d(52, 12 * reflect, 0), this);
        }
        else {
            switch (preloadGlobal) {
                case TOP:
                    robot.goToPoint(new Pose2d(-32, 32 * reflect, 0), this);

                    start = System.currentTimeMillis();
                    robot.intake.actuationDown();
                    while(System.currentTimeMillis() - start <= 500) {
                        robot.update();
                    }
                    start = System.currentTimeMillis();
                    while(System.currentTimeMillis() - start <= 2000) {
                        robot.intake.reverse();
                        robot.update();
                    }
                    robot.intake.off();

                    robot.goToPoint(new Pose2d(-36, 12 * reflect, 0), this);
                    robot.goToPoint(new Pose2d(36, 12 * reflect, 0), this);
                    robot.goToPoint(new Pose2d(48, 42*reflect, 0), this);
                    robot.depositAt(8, 6 * reflect);
                    robot.dunk(1);
                    break;
                case CENTER:
                    robot.goToPoint(new Pose2d(-32, 32 * reflect, Math.toRadians(90 * reflect)), this);

                    start = System.currentTimeMillis();
                    robot.intake.actuationDown();
                    while(System.currentTimeMillis() - start <= 500) {
                        robot.update();
                    }
                    start = System.currentTimeMillis();
                    while(System.currentTimeMillis() - start <= 2000) {
                        robot.intake.reverse();
                        robot.update();
                    }
                    robot.intake.off();

                    robot.goToPoint(new Pose2d(-36, 12 * reflect, 0), this);
                    robot.goToPoint( new Pose2d(36, 12 * reflect, 0), this);
                    robot.goToPoint(new Pose2d(48, 36 * reflect, 0), this);
                    robot.depositAt(8,0);
                    robot.dunk(1);
                    break;
                case BOTTOM:
                    robot.goToPoint(new Pose2d(-32, 32 * reflect, Math.PI), this);

                    start = System.currentTimeMillis();
                    robot.intake.actuationDown();
                    while(System.currentTimeMillis() - start <= 500) {
                        robot.update();
                    }
                    start = System.currentTimeMillis();
                    while(System.currentTimeMillis() - start <= 2000) {
                        robot.intake.reverse();
                        robot.update();
                    }
                    robot.intake.off();

                    robot.goToPoint(new Pose2d(-36, 12 * reflect, 0), this);
                    robot.goToPoint( new Pose2d(36, 12 * reflect, 0), this);
                    robot.goToPoint(new Pose2d(48, 30 * reflect, 0), this);
                    robot.depositAt(8,-6 * reflect);
                    robot.dunk(1);
                    break;
            }
            robot.goToPoint(new Pose2d(36, 12 * reflect, 0), this);
            robot.goToPoint(new Pose2d(52, 12 * reflect, 0), this);
        }
    }
}