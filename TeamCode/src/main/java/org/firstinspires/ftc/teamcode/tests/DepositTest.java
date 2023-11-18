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
    public static Deposit.State state = Deposit.State.DOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.deposit.state = state;

            robot.update();
        }
    }
}
