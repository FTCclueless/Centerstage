package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class MaxSpeedYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        double maxVel = 0;
        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);
            Pose2d relVel = robot.drivetrain.localizer.relCurrentVel;
            double vel = Math.sqrt(Math.pow(relVel.x, 2) + Math.pow(relVel.y, 2));
            maxVel = Math.max(vel, maxVel);
            TelemetryUtil.packet.put("Max vel", maxVel);
            robot.update();
        }
    }
}
