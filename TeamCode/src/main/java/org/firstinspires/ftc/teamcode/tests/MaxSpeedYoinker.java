package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

//@Disabled
@TeleOp
public class MaxSpeedYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        double maxXSpeed = 0;
        double maxYSpeed = 0;
        double maxHeadingSpeed = 0;

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);
            Pose2d velocity = robot.drivetrain.localizers[0].getRelativePoseVelocity();

            if (velocity.x > maxXSpeed) {
                maxXSpeed = velocity.x;
            }
            if (velocity.y > maxYSpeed) {
                maxYSpeed = velocity.y;
            }
            if (velocity.heading > maxHeadingSpeed) {
                maxHeadingSpeed = velocity.heading;
            }

            TelemetryUtil.packet.put("maxXSpeed", maxXSpeed);
            TelemetryUtil.packet.put("maxYSpeed", maxYSpeed);
            TelemetryUtil.packet.put("maxHeadingSpeed", maxHeadingSpeed);

            robot.update();
        }
    }
}
