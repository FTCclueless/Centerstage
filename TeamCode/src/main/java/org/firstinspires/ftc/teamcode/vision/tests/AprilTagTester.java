package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Disabled
@TeleOp
public class AprilTagTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.AUTO;
        Globals.isRed = true;
        Vision vision = new Vision(hardwareMap, telemetry, true, false, true);
        Robot robot = new Robot(hardwareMap, vision);
        robot.drivetrain.setPoseEstimate(new Pose2d(-36.911, -60.25, -Math.PI / 2));

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
