package org.firstinspires.ftc.teamcode.opmodes.autosSplines;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

public class GroundPreloadAuto extends LinearOpMode {
    private boolean up = false; // Is on top side of field
    private boolean blue = false;
    enum PreloadGlobal {
        TOP,
        CENTER,
        BOTTOM
    }

    PreloadGlobal preloadGlobal = PreloadGlobal.CENTER;

    @Override
    public void runOpMode() throws InterruptedException {
        TeamPropDetectionPipeline.TeamPropLocation team_prop_location = TeamPropDetectionPipeline.TeamPropLocation.CENTER;
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.AUTO;

        Vision vision = new Vision();
        TeamPropDetectionPipeline teamPropDetectionPipeline;

        // TODO: add initalization sequence
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, true);
        vision.initCamera(hardwareMap, teamPropDetectionPipeline);

        while (opModeInInit()) {
            robot.intake.actuationUp();
            team_prop_location = teamPropDetectionPipeline.getTeamPropLocation();

            robot.update();

            telemetry.addData("leftAvg", teamPropDetectionPipeline.leftAvg);
            telemetry.addData("centerAvg", teamPropDetectionPipeline.centerAvg);
            telemetry.addData("rightAvg", teamPropDetectionPipeline.rightAvg);
            telemetry.addData("propLocation: ", team_prop_location);
            telemetry.update();
        }

        int reflect = blue ? 1 : -1; // Reflect for blue side

        Pose2d pose = null;
        if (up) {
            pose = new Pose2d(12, -63 * reflect, Math.toRadians(90 * reflect));
        } else {
            pose = new Pose2d(-36, -63 * reflect, Math.toRadians(90 * reflect)); //  up and down are mixed together
        }


        robot.drivetrain.localizer.setPoseEstimate(pose);

        if (team_prop_location == TeamPropDetectionPipeline.TeamPropLocation.CENTER) {
            preloadGlobal = PreloadGlobal.CENTER;
        } else if (team_prop_location == TeamPropDetectionPipeline.TeamPropLocation.LEFT && blue || team_prop_location == TeamPropDetectionPipeline.TeamPropLocation.RIGHT && !blue) {
            preloadGlobal = PreloadGlobal.TOP;
        }
        else {
            preloadGlobal = PreloadGlobal.BOTTOM;
        }
        Spline preloadSpline = new Spline(pose, 4);

        switch (preloadGlobal) {
            case TOP:
                if (up) {
                    preloadSpline.setReversed(true)
                            .addPoint(15, 35*reflect, 0);
                } else {
                    preloadSpline
                            .setReversed(true)
                            .addPoint(-28, 35 * reflect, 0);
                }
                break;
            case CENTER:
                if (up) {
                    preloadSpline
                            .setReversed(true)
                            .addPoint(7.5, 35.5 * reflect, Math.toRadians(-90 * reflect));
                } else {
                    preloadSpline
                            .setReversed(true)
                            .addPoint(-32, 35.5 * reflect, Math.toRadians(-90 * reflect));
                }
                break;
            case BOTTOM:
                if (up) {
                    preloadSpline
                            .setReversed(true)
                            .addPoint(14.5, 35 * reflect, Math.PI);
                } else {
                    preloadSpline
                            .setReversed(true)
                            .addPoint(-36, 35 * reflect, Math.PI);
                }
                break;
        }

        //use a gotopoint to leave preload

        Spline parkSpline = new Spline(pose.x, pose.y, 0, 4)
                .addPoint(53, 55, 0);

        if (!up) {
            if (team_prop_location == TeamPropDetectionPipeline.TeamPropLocation.CENTER) {
                robot.goToPoint(new Pose2d(-53, 55 * reflect, 0), this);

                robot.goToPoint(new Pose2d(53, 58*reflect, 0), this);
            }
            else {
                robot.goToPoint(new Pose2d(-48, 55*reflect, 0), this);
                robot.goToPoint(new Pose2d(53, 55 * reflect, 0), this);
            }
        }
        else {
            robot.goToPoint(new Pose2d(12, 60 * reflect, 0), this);
            robot.goToPoint(new Pose2d(36, 60 * reflect, 0), this);
            robot.goToPoint(new Pose2d(53, 60 * reflect, 0), this);
        }

        waitForStart();

        robot.followSpline(preloadSpline, this);

        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start <= 6000) {
            robot.update();
        }

        robot.goToPoint(pose.x, pose.y, 0, this);


        start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start <= 6000) {
            robot.update();
        }

        robot.followSpline(parkSpline, this);


    }
}

