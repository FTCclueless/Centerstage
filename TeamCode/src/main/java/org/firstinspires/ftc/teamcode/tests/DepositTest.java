package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;

@Config
@TeleOp
public class DepositTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap, robot.hardwareQueue, robot.sensors, true);
        waitForStart();

        while (!isStopRequested()) {
            deposit.update();
        }
    }
}
