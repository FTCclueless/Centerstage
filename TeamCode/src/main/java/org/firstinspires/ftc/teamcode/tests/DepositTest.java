package org.firstinspires.ftc.teamcode.tests;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;


@Disabled
@Config
@TeleOp
public class DepositTest extends LinearOpMode {
    public static double targetHeight = 10;
    public static double targetX = 15;
    public static double heading = 180;
    public static boolean startDeposit = false;
    public static boolean startRetract = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.AUTO;
        ButtonToggle btx = new ButtonToggle();
        ButtonToggle bty = new ButtonToggle();

        waitForStart();

        robot.drivetrain.setPoseEstimate(new Pose2d(0,0,Math.toRadians(heading)));
        boolean depo = false;

        while (opModeIsActive()) {
            if (btx.isClicked(gamepad1.x) || startDeposit) {
                depo = true;
                startDeposit = false;
                robot.deposit.state = Deposit.State.START_DEPOSIT;
            }

            if (depo) {
                robot.deposit.depositAt(targetHeight, targetX);
            }

            Log.e("gamepady", " " + gamepad1.y);
            if (bty.isClicked(gamepad1.y) || startRetract) {
                System.out.println("I NEED TO DO SOMETHING");
                robot.releaseTwo();
                robot.deposit.state = Deposit.State.RETRACT;
                depo = false;
                startRetract = false;
            }

            robot.update();
        }
    }
}
