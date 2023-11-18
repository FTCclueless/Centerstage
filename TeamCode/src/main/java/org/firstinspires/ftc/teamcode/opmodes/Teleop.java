package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Intake intake = robot.intake;

        robot.drivetrain.setMinPowersToOvercomeFriction();

        Globals.RUNMODE = RunMode.TELEOP;

        // Button Toggle naming convention = BUTTON_DRIVER (for example, button a for driver 1 should be called a_1)

        // DRIVER 1
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();

        // DRIVER 2
        ButtonToggle dpadUp_2 = new ButtonToggle();
        ButtonToggle dpadDown_2 = new ButtonToggle();
        ButtonToggle rightTrigger_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle b_2 = new ButtonToggle();

        Vector3 depoPos = new Vector3(0, 0, 0); /* I messed up the coord systems so badly :( */

        waitForStart();

        boolean depoFlag = false;

        while (!isStopRequested()) {

            // adjusting angle of actuation
            if (gamepad1.right_bumper) {
                robot.intake.actuation.setTargetAngle(robot.intake.actuation.getCurrentAngle() + 5, 1.0);
            }

            if (gamepad1.right_trigger > 0.2) {
                robot.intake.actuation.setTargetAngle(robot.intake.actuation.getCurrentAngle() - 5, 1.0);
            }

            /*if (dpadUp_1.isClicked(gamepad1.dpad_up)) {
                depoFlag = true;
                depoPos.y++;
            }
            if (dpadDown_1.isClicked(gamepad1.dpad_down)) {
                depoFlag = true;
                depoPos.y--;
            }*/

            // driver B adjusting deposit position
            depoPos.x += gamepad2.right_stick_y * 0.2;

            if (dpadUp_2.isClicked(gamepad2.dpad_up)) {
                depoPos.z+=3;
            }
            if (dpadDown_2.isClicked(gamepad2.dpad_down)) {
                depoPos.z-=3;
            }
            depoPos.z += gamepad2.left_stick_y;

            // trigger / retract deposit
            if (a_2.isClicked(gamepad2.a)) {
                depoFlag = true;
            }
            if (b_2.isClicked(gamepad2.b)) {
                depoFlag = false;
                robot.deposit.retract();
            }

            if (depoFlag) {
                robot.deposit.depositAt(depoPos.z, depoPos.y);
            }

            // toggle intake on and off
            if (x_1.isClicked(gamepad1.x)) {
                if (intake.state == Intake.State.ON || intake.state == Intake.State.REVERSED) {
                    intake.off();
                } else if (intake.state == Intake.State.OFF) {
                    intake.on();
                }
            }

            // reverse intake
            if (b_1.isClicked(gamepad1.b)) {
                robot.intake.reverse();
            }

            // dunking
            if (rightTrigger_2.isClicked(gamepad2.right_trigger > 0.2)) {
                robot.deposit.dunk(1);
                depoPos = new Vector3(2,0,10);
            }

            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
