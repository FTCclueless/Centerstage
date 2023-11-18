package org.firstinspires.ftc.teamcode.tests;

import android.widget.Button;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Config
@TeleOp
public class DepositTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        ButtonToggle btx = new ButtonToggle();
        ButtonToggle bty = new ButtonToggle();

        waitForStart();

        robot

/*
        while (opModeIsActive()) {
            robot.intake.on();

            if (btx.isClicked(gamepad1.x)) {
                robot.deposit.state = Deposit.State.START_DEPOSIT;
            }

            if (bty.isClicked(gamepad1.y)) {
                System.out.println("I NEED TO DO SOMETHING");
                robot.dunk(1);
                robot.deposit.state = Deposit.State.START_RETRACT;
            }

            robot.update();
        }*/
    }
}
