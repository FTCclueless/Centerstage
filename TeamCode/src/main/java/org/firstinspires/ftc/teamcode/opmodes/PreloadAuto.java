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

@Autonomous(group = "opmodes", name = "Auto (Good luck!)")
public class PreloadAuto extends LinearOpMode {
    enum State {
        START_INTAKE_START,
        START_INTAKE_CYCLE,
        INTAKE,
        START_GO_DEPOSIT_START,
        START_GO_DEPOSIT_CYCLE,
        GO_DEPOSIT,
        DUNK_EM,
        READY
    }

    private State state = State.READY;
    private Drivetrain.State heldDrivetrainState = null;
    private boolean up = false; // Is on top side of field
    private boolean blockedDepositPath = false;
    private boolean changingPaths = true;

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

        Spline initSpline = null;
        Spline leaveSpline = null;
        if (up) {
            robot.drivetrain.setPoseEstimate(new Pose2d(12, 60, Math.toRadians(90)));
        } else {
            robot.drivetrain.setPoseEstimate(new Pose2d(-36, 60, Math.toRadians(90))); // up and down are mixed together
        }

        // Wubba lubba dub dub
        Pose2d pose = robot.drivetrain.getPoseEstimate();
        switch (team_prop_location) {
            case LEFT:
                initSpline = new Spline(pose, 4)
                    .addPoint(new Pose2d(pose.x, pose.y - 24, Math.toRadians(90)));
                leaveSpline = new Spline(initSpline.getLastPoint(), 4)
                        .addPoint(new Pose2d(pose.x, pose.y-48, 0));
                break;
            case CENTER:
                initSpline = new Spline(pose, 4)
                    .addPoint(new Pose2d(pose.x, pose.y - 24, Math.toRadians(0)));
                leaveSpline = new Spline(initSpline.getLastPoint(), 4)
                        .addPoint(new Pose2d(pose.x-24, pose.y-24, Math.toRadians(90)))
                        .addPoint(new Pose2d(pose.x-24, pose.y-48, Math.toRadians(0)));
                break;
            case RIGHT:
                initSpline = new Spline(pose, 4)
                    .addPoint(new Pose2d(pose.x, pose.y - 24, Math.toRadians(-90)));
                leaveSpline = new Spline(initSpline.getLastPoint(), 4)
                        .addPoint(new Pose2d(pose.x, pose.y-48, 0));
                break;
            default:
                // CRASH CRASH BAD BAD!
                RobotLog.e("BAD BAD! CRASH! SOMETHING TERRIBLE HAPPENED! GET HUDSON!");
                break;
        }

        Spline toPark = new Spline(leaveSpline.getLastPoint(), 4)
                .addPoint(new Pose2d(60, 12, 0));

        waitForStart();

        state = up ? State.START_GO_DEPOSIT_START : State.START_INTAKE_START;

        while (opModeIsActive()) {

            robot.update();
        }
    }
}
