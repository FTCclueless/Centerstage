package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
@Config
public class PurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        Spline path = new Spline(0, 0,0, 3);
        path.addPoint(40, 30, Math.PI/2);
        path.addPoint(0,60,Math.PI);

        waitForStart();

        robot.drivetrain.setStop(true);
        robot.drivetrain.setFinalAdjustment(true);
        robot.followSpline(path, this::opModeIsActive);
    }
}
