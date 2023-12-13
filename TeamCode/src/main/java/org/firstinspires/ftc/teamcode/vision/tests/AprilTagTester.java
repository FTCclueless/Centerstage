package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class AprilTagTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap, telemetry, true, true, true);
        Robot robot = new Robot(hardwareMap, vision);
        robot.drivetrain.localizer.setPoseEstimate(new Pose2d(48,-48, Math.toRadians(180)));

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
