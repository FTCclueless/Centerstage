package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;


@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
//        robot.drivetrain.setPoseEstimate(new Pose2d(-35.411, -60.5, 0));
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
        Globals.RUNMODE = RunMode.TESTER;

        waitForStart();

        while(!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
