package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
@Config
public class GoToPointTester extends LinearOpMode {
    public static double x = 0;
    public static double y = 0;
    public static double h = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        drivetrain.setPoseEstimate(new Pose2d(0,0, Math.toRadians(0)));

        waitForStart();

        while (!isStopRequested()) {
            robot.goToPoint(new Pose2d(x, y, Math.toRadians(h)),this, true, true);
            robot.update();
        }
    }
}
