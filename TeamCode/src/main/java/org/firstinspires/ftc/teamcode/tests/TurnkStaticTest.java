package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@Disabled
@TeleOp
@Config
public class TurnkStaticTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;

        waitForStart();

        drivetrain.setMotorPowers(0.34759999999997804, 0.5678999999999538, 0.38109999999997435, 0.3940999999999729);

        while (!isStopRequested()) {
            robot.update();
        }

    }
}
