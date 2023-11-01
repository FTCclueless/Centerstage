package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Config
@TeleOp
public class DepositTest extends LinearOpMode{
    public static double targetH = 10;
    public static double targetY = 0;
    public static double xOffset = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap, robot.hardwareQueue, robot.sensors);
        waitForStart();

        deposit.setTargetBoard(new Pose2d(10,0,0));

        while (!isStopRequested()) {
            deposit.depositAt(targetH, targetY, xOffset);
            robot.update();
        }
    }
}
