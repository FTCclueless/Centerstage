package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

// The following auto does NOT do the init

@Autonomous(group = "opmodes", name = "Auto (Good luck!)")
public class Auto extends LinearOpMode {
    enum State {
        START_INTAKE,
        INTAKE,
        START_GO_DEPOSIT,
        GO_DEPOSIT,
        DUNK_EM,
        READY
    }

    private State state = State.READY;
    private Drivetrain.State heldDrivetrainState = null;
    private boolean up = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        state = State.START_INTAKE;
        if (up) {
            robot.drivetrain.setPoseEstimate(new Pose2d(-36, 60, Math.toRadians(90)));
        } else {
            robot.drivetrain.setPoseEstimate(new Pose2d(12, 60, Math.toRadians(90)));
        }

        Spline toIntake = new Spline(robot.drivetrain.getPoseEstimate(), 4);
        // An alliance is blocking us from going to the board
        // TODO: unsure path
        Spline blockedToDeposit = new Spline(toIntake.getLastPoint(), 4)
            .setReversed(true)
            .addPoint(new Pose2d(-60, 12, Math.toRadians(45)))
            .addPoint(new Pose2d(36, 12, Math.toRadians(90)))
            .addPoint(new Pose2d(48, 36, Math.toRadians(90)));
        Spline toDeposit = new Spline(toIntake.getLastPoint(), 4)
            .setReversed(true)
            .addPoint(new Pose2d(48, 36, Math.toRadians(90)));

        while (opModeIsActive()) {
            switch (state) {
                case START_INTAKE:
                    robot.drivetrain.setCurrentPath(toIntake);
                    state = State.GO_DEPOSIT;
                    break;

                case INTAKE:
                    if (/* robot detected */ false) {
                        // Store state so we can restore it after
                        heldDrivetrainState = robot.drivetrain.state;
                        robot.drivetrain.state = Drivetrain.State.BRAKE;
                    } else if (heldDrivetrainState != null) {
                        robot.drivetrain.state = heldDrivetrainState;
                    }

                    if (/* test for pixels intaken */ false) {
                        state = State.START_GO_DEPOSIT;
                    }
                    break;

                case START_GO_DEPOSIT:
                    robot.drivetrain.setCurrentPath(/* robot detected */ false ? blockedToDeposit : toDeposit);
                    state = State.GO_DEPOSIT;
                    break;

                case GO_DEPOSIT:
                    Pose2d pose = robot.drivetrain.localizer.getPoseEstimate();
                    // Switch auto paths if robot detected and we are outside of special zone
                    if (pose.x > 4 || pose.x < -30) {
                        if (/* robot detected */ false) {
                            // Switch state TODO:
                        }
                    } else if (/* robot detected */ false) {
                        // Pause
                    }
                    break;
            }
            robot.update();
        }
    }
}
