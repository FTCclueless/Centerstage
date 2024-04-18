package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class SlideTuner extends LinearOpMode {
    public static double targetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            robot.deposit.slides.setTargetLength(targetPos);
            robot.update();
        }
    }
}
