package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
public class LocalizationCompTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.drivetrain.setPoseEstimate(new Pose2d(0.0, 0.0, Math.toRadians(180)));
        while (opModeInInit()) {
            robot.update();
        }

        waitForStart();

        robot.goToPoint(new Pose2d(-12.0, -24.0, Math.toRadians(180)), this, false, false, 0.75);

        for(int i = 0; i < 5; i++) {
            robot.followSpline(
                new Spline(
                    new Pose2d(-12, -24, Math.toRadians(180)),
                    4
                )
                    .addPoint(new Pose2d(-24, -20, Math.toRadians(180)), 0.75)
                    .addPoint(new Pose2d(-78, -44, Math.toRadians(270)), 0.75),
                this::opModeIsActive
            );
            //robot.splineToPoint(new Pose2d(-78.0, -44.0, Math.toRadians(270)), this, true, true, 0.75, false);
            robot.splineToPoint(new Pose2d(-12.0, -24.0, Math.toRadians(180)), this, false, false, 0.75, true);
        }

        robot.goToPoint(new Pose2d(0.0, 0.0, Math.toRadians(180)), this, true, true, 0.75);

        robot.drivetrain.state = Drivetrain.State.IDLE;

        while (!isStopRequested()){
            robot.update();
        }

    }
}
