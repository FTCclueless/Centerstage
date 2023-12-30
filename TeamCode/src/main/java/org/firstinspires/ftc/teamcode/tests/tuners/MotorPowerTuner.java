package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@TeleOp
@Config
public class MotorPowerTuner extends LinearOpMode {
    public static double lf = 0.0;
    public static double lr = 0.0;
    public static double rr = 0.0;
    public static double rf = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.setMinPowersToOvercomeFriction();
            robot.drivetrain.setMotorPowers(lf, lr, rr, rf);
            robot.update();
        }
    }
}
