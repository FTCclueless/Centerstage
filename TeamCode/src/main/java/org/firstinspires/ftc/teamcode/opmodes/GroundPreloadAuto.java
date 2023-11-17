package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

// The following auto does NOT do the init

@Autonomous(group = "opmodes", name = "Ground Preload Auto")
public class GroundPreloadAuto extends LinearOpMode {
    private boolean up = true; // Is on top side of field
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

        Pose2d pose = null;
        if (up) {
            pose = new Pose2d(12, -60, Math.toRadians(-90));
        } else {
            pose = new Pose2d(-36, -60, Math.toRadians(90)); //  up and down are mixed together
        }

        robot.drivetrain.localizer.setPoseEstimate(pose);

        if (blue) {
            pose.y *= -1;
            pose.heading *= -1;
        }

        waitForStart();

        int reflect = blue ? 1 : -1; // Reflect for blue side

        team_prop_location = TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER;

        if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.CENTER) {
            preloadGlobal = PreloadGlobal.CENTER;
        } else if (team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.LEFT && blue || team_prop_location == TeamPropDetectionPipeline.TEAM_PROP_LOCATION.RIGHT && !blue) {
            preloadGlobal = PreloadGlobal.TOP;
        }
        else {
            preloadGlobal = PreloadGlobal.BOTTOM;
        }
        long start = 0;

        switch (preloadGlobal) {
            case TOP:
                if (up) {
                    robot.goToPoint(new Pose2d(12, 32 * reflect, 0), this);
                } else {
                    robot.goToPoint(new Pose2d(-32, 32 * reflect, 0), this);
                }

                break;
            case CENTER:
                if (up) {
                    robot.goToPoint(new Pose2d(12, 36 * reflect, Math.toRadians(90 * reflect)), this);
                } else {
                    robot.goToPoint(new Pose2d(-32, 32 * reflect, Math.toRadians(90 * reflect)), this);
                }
                break;
            case BOTTOM:
                if (up) {
                    robot.goToPoint(new Pose2d(12, 32 * reflect, Math.PI), this);
                } else {
                    robot.goToPoint(new Pose2d(-32, 32 * reflect, Math.PI), this);
                }

                break;

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

        robot.goToPoint(new Pose2d(pose.x, 12 * reflect, 0), this);
        robot.goToPoint(new Pose2d(52, 12 * reflect, 0), this);

        robot.intake.actuationSinglePixel();
    }
}
