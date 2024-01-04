package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector3;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Intake intake = robot.intake;
        Hang hang = robot.hang;

        Globals.RUNMODE = RunMode.TELEOP;
        robot.deposit.state = Deposit.State.INTAKE;

        // Button Toggle naming convention = BUTTON_DRIVER (for example, button a for driver 1 should be called a_1)

        // DRIVER 1
        ButtonToggle rightBumper_1 = new ButtonToggle();
        ButtonToggle leftBumper_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle y_1 = new ButtonToggle();
        ButtonToggle a_1 = new ButtonToggle();
        ButtonToggle leftTrigger_1 = new ButtonToggle();
        ButtonToggle leftTrigger_1_double = new ButtonToggle();
        ButtonToggle left_dpad_1 = new ButtonToggle();

        // DRIVER 2
        ButtonToggle dpadUp_2 = new ButtonToggle();
        ButtonToggle dpadDown_2 = new ButtonToggle();
        ButtonToggle dpadLeft_2 = new ButtonToggle();
        ButtonToggle dpadRight_2 = new ButtonToggle();
        ButtonToggle leftTrigger_2 = new ButtonToggle();
        ButtonToggle rightTrigger_2 = new ButtonToggle();
        ButtonToggle leftBumper_2 = new ButtonToggle();
        ButtonToggle rightBumper_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle x_2 = new ButtonToggle();
        ButtonToggle y_2 = new ButtonToggle();

        int pixelIndex = 0;
        Vector3 depoPos = new Vector3(15, 0, 10);

        robot.airplane.hold();
        robot.droppers.retractBoth();
        robot.drivetrain.setPoseEstimate(Globals.AUTO_ENDING_POSE);

        waitForStart();

        boolean depoFlag = false;

        while (!isStopRequested()) {
            // ------------------- DRIVER 1 CONTROLS -------------------

            // adjusting angle of actuation
            if (rightBumper_1.isClicked(gamepad1.right_bumper)) {
                if (intake.isActuationUp()) {
                    robot.intake.actuationFullyDown();
                } else {
                    robot.intake.actuationFullyUp();
                }
            }

            // lift actuation if we are reversing intake
            if (intake.state == Intake.State.REVERSED || intake.state == Intake.State.REVERSE_FOR_TIME) {
                robot.intake.actuationFullyUp();
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
                if (intake.state == Intake.State.ON || intake.state == Intake.State.OFF) {
                    intake.reverse();
                } else if (intake.state == Intake.State.REVERSED) {
                    intake.off();
                }
            }

            // reverse intake but only for a 0.5 secs
            if (leftTrigger_1_double.isClicked(gamepad1.left_trigger > 0.2) && robot.deposit.state == Deposit.State.INTAKE) {
                robot.intake.reverseForSomeTime(500);
            }

            // trigger deposit (both)
            if (leftBumper_1.isClicked(gamepad1.left_bumper)) {
                Globals.NUM_PIXELS = 2;
                depoFlag = true;
                if (robot.deposit.state == Deposit.State.DEPOSIT) {
                    robot.deposit.releaseOne();
                }
            }

            // hang arms
            if (left_dpad_1.isClicked(gamepad1.dpad_left)) {
                robot.hang.nextHangState();
            }

            // hanging mechanism
            if (gamepad1.y && robot.hang.doingHang()) {
                hang.on();
            } else if (gamepad1.a && robot.hang.doingHang()) {
                hang.reverse();
            } else {
                hang.off();
            }

            // adjust slides height
            if (gamepad1.y && !robot.hang.doingHang()) {
                depoPos.z+=0.5;
            }
            if (gamepad1.a && !robot.hang.doingHang()) {
                depoPos.z-=0.5;
            }

            // release one pixel (both)
            if (leftTrigger_1.isClicked(gamepad1.left_trigger > 0.2) && !robot.deposit.release.readyToRetract() && robot.deposit.state == Deposit.State.DEPOSIT) {
                robot.deposit.releaseOne();
            }

            // airplane (both)
            if (gamepad1.dpad_up) {
                robot.airplane.release();
            }

            // driving
            robot.drivetrain.drive(gamepad1);






            // ------------------- DRIVER 2 CONTROLS -------------------

            // driver B adjusting deposit position
            if (dpadUp_2.isClicked(gamepad2.dpad_up)) {
                depoPos.z+=3;
            }
            if (dpadDown_2.isClicked(gamepad2.dpad_down)) {
                depoPos.z-=3;
            }
            depoPos.x -= gamepad2.right_stick_y*0.2;
            depoPos.z -= gamepad2.left_stick_y*0.2;

            // trigger deposit
            if (rightBumper_2.isClicked(gamepad2.right_bumper)) {
                Globals.NUM_PIXELS = 2;
                depoFlag = true;
                if (robot.deposit.state == Deposit.State.DEPOSIT) {
                    robot.deposit.releaseOne();
                }
            }

            if (depoFlag) {
                robot.deposit.depositAt(depoPos.z, depoPos.x);
            }

            TelemetryUtil.packet.put("depoz: ", depoPos.z);
            TelemetryUtil.packet.put("depoy: ", depoPos.y);
            TelemetryUtil.packet.put("depox", depoPos.x);
            TelemetryUtil.packet.put("depoFlag", depoFlag);

            // dunking (both)
            if (rightTrigger_2.isClicked(gamepad2.right_trigger > 0.2) && !robot.deposit.release.readyToRetract() && robot.deposit.state == Deposit.State.DEPOSIT) {
                robot.deposit.releaseOne();
            }

            if (leftTrigger_2.isClicked(gamepad2.left_trigger > 0.2) && !robot.deposit.release.readyToRetract() && robot.deposit.state == Deposit.State.DEPOSIT) {
                robot.deposit.releaseTwo();
            }

            // retract
            if ((x_2.isClicked(gamepad2.x) || robot.deposit.release.readyToRetract()) && (robot.deposit.state == Deposit.State.DEPOSIT || robot.deposit.state == Deposit.State.FINISH_DEPOSIT)) {
                robot.deposit.retract();
                depoFlag = false;
            }

            // airplane release (both)
            if (gamepad2.b) {
                robot.airplane.release();
            }

            // adjusting actuation angle
            if (y_2.isClicked(gamepad2.y) && !hang.doingHang()) {
                pixelIndex++;
                pixelIndex = Utils.minMaxClipInt(pixelIndex, 0, 4);
                intake.setActuationHeight(pixelIndex);
            }

            if (a_2.isClicked(gamepad2.a) && !hang.doingHang()) {
                pixelIndex--;
                pixelIndex = Utils.minMaxClipInt(pixelIndex, 0, 4);
                intake.setActuationHeight(pixelIndex);
            }

            // hang arms
            if (leftBumper_2.isClicked(gamepad2.left_bumper)) {
                robot.hang.nextHangState();
            }

            // hanging mechanism
            if (gamepad2.y && robot.hang.doingHang()) {
                hang.on();
            } else if (gamepad2.a && robot.hang.doingHang()) {
                hang.reverse();
            } else {
                hang.off();
            }

            telemetry.addData("Pixel Height", pixelIndex+1);
            telemetry.update();

            // update robot
            robot.update();
        }
    }
}
