package org.firstinspires.ftc.teamcode.opmodes;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
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
        robot.drivetrain.setMinPowersToOvercomeFriction();

        Globals.RUNMODE = RunMode.TELEOP;
        ButtonToggle btDpadUp =       new ButtonToggle();
        ButtonToggle btDpadDown =     new ButtonToggle();
        ButtonToggle btDpadLeft =     new ButtonToggle();
        ButtonToggle btDpadRight =    new ButtonToggle();
        ButtonToggle btLeftTrigger =  new ButtonToggle();
        ButtonToggle btLeftBumper =   new ButtonToggle();
        ButtonToggle btRightTrigger = new ButtonToggle();
        ButtonToggle btRightBumper =  new ButtonToggle();
        ButtonToggle btY =            new ButtonToggle();
        ButtonToggle btA =            new ButtonToggle();
        ButtonToggle btX =            new ButtonToggle();
        ButtonToggle btB =            new ButtonToggle();
        Vector3 depoPos =             new Vector3(0, 0, 0); /* I messed up the coord systems so badly :( */

        waitForStart();

        while (!isStopRequested()) {
            // Weemawaeoeoewa :)
            if (btRightBumper.isClicked(gamepad1.right_bumper)) {
                robot.intake.setActuationPixelHeight((int) (robot.intake.getIntakeActuationOffset() + 1));
            }
            if (btRightTrigger.isClicked(gamepad1.right_trigger > 0.2)) {
                robot.intake.setActuationPixelHeight((int) (robot.intake.getIntakeActuationOffset() - 1));
            }

            boolean depoFlag = false;
            if (btDpadUp.isClicked(gamepad1.dpad_up)) {
                depoFlag = true;
                depoPos.y++;
            }
            if (btDpadDown.isClicked(gamepad1.dpad_down)) {
                depoFlag = true;
                depoPos.y--;
            }
            if (btDpadLeft.isClicked(gamepad1.dpad_left)) {
                depoFlag = true;
                depoPos.x--;
            }
            if (btDpadRight.isClicked(gamepad1.dpad_right)) {
                depoFlag = true;
                depoPos.x++;
            }

            if (btLeftBumper.isClicked(gamepad1.left_bumper)) {
                depoFlag = true;
                depoPos.z++;
            }
            if (btLeftTrigger.isClicked(gamepad1.left_trigger > 0.2)) {
                depoFlag = true;
                depoPos.z--;
            }

            if (depoFlag) {
                robot.deposit.depositAt(depoPos.y, depoPos.x, depoPos.z);
            }

            if (btX.isClicked(gamepad1.x)) {
                robot.intake.toggle();
            }

            if (btB.isClicked(gamepad1.b)) {
                robot.deposit.dunk(1);
            }

            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
