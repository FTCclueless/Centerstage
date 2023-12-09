package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Intake intake = robot.intake;
        Hang hang = robot.hang;

        Globals.RUNMODE = RunMode.TELEOP;
        robot.deposit.state = Deposit.State.DOWN;
        robot.drivetrain.setMinPowersToOvercomeFriction();

        // Button Toggle naming convention = BUTTON_DRIVER (for example, button a for driver 1 should be called a_1)

        // DRIVER 1
        ButtonToggle rightBump = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();

        // DRIVER 2
        ButtonToggle dpadUp_2 = new ButtonToggle();
        ButtonToggle dpadDown_2 = new ButtonToggle();
        ButtonToggle dpadLeft_2 = new ButtonToggle();
        ButtonToggle dpadRight_2 = new ButtonToggle();
        ButtonToggle rightTrigger_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();

        Vector3 depoPos = new Vector3(10, 0, 10); /* I messed up the coord systems so badly :( */

        waitForStart();

        boolean depoFlag = false;

        while (!isStopRequested()) {

            // adjusting angle of actuation
            if (rightBump.isToggled(gamepad1.right_bumper)) {
                robot.intake.actuationUp();
            } else {
                robot.intake.actuationDown();
            }

            // driver B adjusting deposit position
            if (dpadLeft_2.isClicked(gamepad2.dpad_left)) {
                depoFlag = true;
                depoPos.y+=6;
            }
            if (dpadRight_2.isClicked(gamepad2.dpad_right)) {
                depoFlag = true;
                depoPos.y-=6;
            }
            if (dpadUp_2.isClicked(gamepad2.dpad_up)) {
                depoPos.z+=6;
            }
            if (dpadDown_2.isClicked(gamepad2.dpad_down)) {
                depoPos.z-=6;
            }
            depoPos.x -= gamepad2.left_stick_y*0.2;
            depoPos.y += gamepad2.left_stick_x*0.2;
            depoPos.z -= gamepad2.left_stick_y*0.2;

            // trigger deposit
            if (a_2.isClicked(gamepad2.a)) {
                depoFlag = true;
            }

            if (depoFlag) {
                TelemetryUtil.packet.put("depoz: ", depoPos.z);
                TelemetryUtil.packet.put("depoy: ", depoPos.y);
                TelemetryUtil.packet.put("depox", depoPos.x);
                robot.deposit.depositAt(depoPos.z, depoPos.y, depoPos.x);
            }

            TelemetryUtil.packet.put("depoFlag", depoFlag);

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
            if (rightTrigger_2.isClicked(gamepad2.right_trigger > 0.2 && robot.deposit.state == Deposit.State.FINISH_DEPOSIT)) {
                robot.deposit.dunk(1);
                depoPos = new Vector3(2,0,10);
                depoFlag = false;
            }

            if (a_2.isClicked(gamepad2.a)) {
                robot.depositAt(depoPos.z, depoPos.y);
            }

            // hanging mechanism
            if (gamepad1.y) {
                hang.on();
            } else if (gamepad1.a) {
                hang.reverse();
            } else {
                hang.off();
            }

            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
