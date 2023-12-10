package org.firstinspires.ftc.teamcode.tests;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp
public class DepositTest extends LinearOpMode {
    public static double height = 10;
    public static double targetY = 0;
    public static double xError = 15;
    public static double heading = 180;
    public static boolean startDeposit = false;
    public static boolean startRetract = false;
    public static boolean close = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.AUTO;
        ButtonToggle btx = new ButtonToggle();
        ButtonToggle bty = new ButtonToggle();

        waitForStart();

        robot.drivetrain.setPoseEstimate(new Pose2d(0,0,Math.toRadians(heading)));
        robot.deposit.setTargetBoard(new Pose2d(xError,0,0));
        boolean depo = false;

        while (opModeIsActive()) {
            //robot.intake.on();

            if (btx.isClicked(gamepad1.x) || startDeposit) {
                depo = true;
                startDeposit = false;
                robot.deposit.state = Deposit.State.START_DEPOSIT;
            }

            if (depo) {
                robot.deposit.depositAt(height, targetY, xError);
            }

            if (close) {
                robot.deposit.dunker.close();
                close = false;
            }

            Log.e("gamepady", " " + gamepad1.y);
            if (bty.isClicked(gamepad1.y) || startRetract) {
                System.out.println("I NEED TO DO SOMETHING");
                robot.dunk(1);
                robot.deposit.state = Deposit.State.START_RETRACT;
                depo = false;
                startRetract = false;
            }

            robot.update();
        }
    }
}
