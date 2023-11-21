package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

// The following auto does NOT do the init

@Autonomous(group = "opmodes", name = "RED Ground Preload Auto Down")
public class REDGroundPreloadAutoDown extends LinearOpMode {
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
        TeamPropDetectionPipeline.TEAM_PROP_LOCATION team_prop_location = TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER;
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
            pose = new Pose2d(12, -63, Math.toRadians(90 * reflect));
        } else {
            pose = new Pose2d(-36, -63, Math.toRadians(90 * reflect)); //  up and down are mixed together
        }

        if (blue) {
            pose.y *= -1;
        }

        robot.drivetrain.localizer.setPoseEstimate(pose);

        waitForStart();

        if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER) {
            preloadGlobal = PreloadGlobal.CENTER;
        } else if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.LEFT && blue || team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.RIGHT && !blue) {
            preloadGlobal = PreloadGlobal.TOP;
        }
        else {
            preloadGlobal = PreloadGlobal.BOTTOM;
        }

        switch (preloadGlobal) {
            case TOP:
                if (up) {
                    robot.goToPoint(new Pose2d(15, 35 * reflect, 0), this);
                } else {
                    robot.goToPoint(new Pose2d(-28, 35 * reflect, 0), this);
                }

                break;
            case CENTER:
                if (up) {
                    robot.goToPoint(new Pose2d(7.5, 35.5 * reflect, Math.toRadians(-90 * reflect)), this);
                } else {
                    robot.goToPoint(new Pose2d(-32, 35.5 * reflect, Math.toRadians(-90 * reflect)), this);
                }
                break;
            case BOTTOM:
                if (up) {
                    robot.goToPoint(new Pose2d(14.5, 35 * reflect, Math.PI), this);
                } else {
                    robot.goToPoint(new Pose2d(-36, 35 * reflect, Math.PI), this);
                }
                break;
        }

        long start = System.currentTimeMillis();
//        robot.intake.actuationDown();
//        while(System.currentTimeMillis() - start <= 500) {
//            robot.update();
//        }
//        start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start <= 6000) {
            robot.intake.softReverse();
            robot.update();
        }
        robot.intake.off();
        if (!up) {
            if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER) {
                robot.goToPoint(new Pose2d(-53, 55 * reflect, 0), this);
                start = System.currentTimeMillis();
                while(System.currentTimeMillis() - start <= 6000) {
                    robot.update();
                }
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
    }
}