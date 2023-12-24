package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Disabled
@TeleOp
@Config
public class DepositStateTest extends LinearOpMode {
    public static Deposit.State state = Deposit.State.INTAKE;
    public static RunMode runmode = RunMode.TELEOP;
    public static boolean update = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (update) {
                Globals.RUNMODE = runmode;
                robot.deposit.state = state;
                update = false;
            }

            robot.update();
        }
    }
}
