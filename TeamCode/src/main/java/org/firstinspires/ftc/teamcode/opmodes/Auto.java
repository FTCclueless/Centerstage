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
        START_INTAKE_START,
        START_INTAKE_CYCLE,
        INTAKE,
        START_GO_DEPOSIT,
        GO_DEPOSIT,
        DUNK_EM,
        READY
    }

    private State state = State.READY;
    private Drivetrain.State heldDrivetrainState = null;
    private boolean up = false; // Is on top side of field
    private boolean directDepositPath = false;
    private boolean canChangePaths = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();
        state = State.START_INTAKE_START;
        if (up) {
            robot.drivetrain.setPoseEstimate(new Pose2d(-36, 60, Math.toRadians(-90)));
        } else {
            robot.drivetrain.setPoseEstimate(new Pose2d(12, 60, Math.toRadians(-90)));
        }

        Spline toIntakeStart = new Spline(up ? new Pose2d(-36, 36, Math.toRadians(-90)) : new Pose2d(12, 36, Math.toRadians(-90)), 4)
            .addPoint(new Pose2d(-60, 36, Math.toRadians(-90))); // dont wokr brbr

        Spline toIntakeCycle = new Spline(new Pose2d(48, 36, Math.toRadians(90)), 4)
            .setReversed(true)
            .addPoint(new Pose2d(-60, 36, Math.toRadians(90)));

        Spline toIntakeCycleBlocked = new Spline(new Pose2d(48, 36, Math.toRadians(90)), 4)
            .setReversed(true)
            .addPoint(new Pose2d(48, 12, Math.toRadians(90)))
            .addPoint(new Pose2d(-48, 12, Math.toRadians(90)))
            .addPoint(new Pose2d(-60, 36, Math.toRadians(90)));

        // An alliance is blocking us from going to the board
        // TODO: unsure path
        Spline blockedToDeposit = new Spline(toIntakeStart.getLastPoint(), 4)
            .setReversed(true)
            .addPoint(new Pose2d(-60, 12, Math.toRadians(-45)))
            .addPoint(new Pose2d(36, 12, Math.toRadians(0)))
            .addPoint(new Pose2d(48, 36, Math.toRadians(0)));
        Spline toDeposit = new Spline(toIntakeStart.getLastPoint(), 4)
            .setReversed(true)
            .addPoint(new Pose2d(48, 36, Math.toRadians(0)));

        while (opModeIsActive()) {
            switch (state) {
                case START_INTAKE_START:
                    robot.drivetrain.setCurrentPath(toIntakeStart);
                    state = State.INTAKE;
                    break;

                case START_INTAKE_CYCLE:
                    robot.drivetrain.setCurrentPath(/* robot detected */ false ? toIntakeCycleBlocked : toIntakeCycle);
                    state = State.INTAKE;
                    break;

                case INTAKE:
                    if (/* robot detected */ false) {
                        // Store state so we can restore it after
                        heldDrivetrainState = robot.drivetrain.state;
                        robot.drivetrain.state = Drivetrain.State.BRAKE;
                    } else if (heldDrivetrainState != null) {
                        robot.drivetrain.state = heldDrivetrainState;
                        heldDrivetrainState = null;
                    }

                    if (/* test for pixels intaken */ true) {
                        state = State.START_GO_DEPOSIT;
                    }
                    break;

                case START_GO_DEPOSIT:
                    robot.drivetrain.setCurrentPath(/* robot detected */ false ? blockedToDeposit : toDeposit);
                    directDepositPath = false;
                    state = State.GO_DEPOSIT;
                    break;

                case GO_DEPOSIT:
                    Pose2d pose = robot.drivetrain.localizer.getPoseEstimate();
                    // Switch auto paths if robot detected and we are outside of special zone
                    if (pose.x > 4 || pose.x < -30) {
                        if (/* robot detected */ false && canChangePaths) {
                            directDepositPath = !directDepositPath;
                            canChangePaths = false; //todo strafe and then go to spline
                            robot.drivetrain.setCurrentPath(directDepositPath ? blockedToDeposit : toDeposit);
                        } else {
                            canChangePaths = true;
                        }
                    } else if (/* robot detected */ false) {
                        heldDrivetrainState = robot.drivetrain.state;
                        robot.drivetrain.state = Drivetrain.State.BRAKE;
                    } else if (heldDrivetrainState != null) {
                        // Restore state
                        robot.drivetrain.state = heldDrivetrainState;
                        heldDrivetrainState = null;
                    }

                    if (!robot.drivetrain.isBusy())
                        state = State.DUNK_EM;

                    break;

                case DUNK_EM:
                    // Dunk em
                    state = State.START_INTAKE_CYCLE;
                    break;
            }
            robot.update();
        }
    }
}
