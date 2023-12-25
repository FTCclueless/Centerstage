package org.firstinspires.ftc.teamcode.opmodes;

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
        ButtonToggle rightBump_1 = new ButtonToggle();
        ButtonToggle leftBump_1 = new ButtonToggle();
        ButtonToggle x_1 = new ButtonToggle();
        ButtonToggle b_1 = new ButtonToggle();
        ButtonToggle leftTrigger_1 = new ButtonToggle();

        // DRIVER 2
        ButtonToggle dpadUp_2 = new ButtonToggle();
        ButtonToggle dpadDown_2 = new ButtonToggle();
        ButtonToggle dpadLeft_2 = new ButtonToggle();
        ButtonToggle dpadRight_2 = new ButtonToggle();
        ButtonToggle leftTrigger_2 = new ButtonToggle();
        ButtonToggle rightTrigger_2 = new ButtonToggle();
        ButtonToggle rightBumper_2 = new ButtonToggle();
        ButtonToggle a_2 = new ButtonToggle();
        ButtonToggle x_2 = new ButtonToggle();

        Vector3 depoPos = new Vector3(15, 0, 10);

        robot.airplane.hold();
        robot.drivetrain.setPoseEstimate(Globals.AUTO_ENDING_POSE);

        waitForStart();

        boolean depoFlag = false;

        while (!isStopRequested()) {
            // ------------------- DRIVER 1 CONTROLS -------------------

            // adjusting angle of actuation
            if (rightBump_1.isToggled(gamepad1.right_bumper) || intake.state == Intake.State.REVERSED) { // TODO: change this to have more adjustment
                robot.intake.actuationUp();
            } else {
                robot.intake.actuationDown();
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

            // hanging mechanism
            if (gamepad1.y) {
                hang.on();
            } else if (gamepad1.a) {
                hang.reverse();
            } else {
                hang.off();
            }

            // trigger deposit (both)
            if (leftBump_1.isClicked(gamepad1.left_bumper)) {
                Globals.NUM_PIXELS = 2;
                depoFlag = true;
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
            if (a_2.isClicked(gamepad2.a)) {
                Globals.NUM_PIXELS = 2;
                depoFlag = true;
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
            if (gamepad2.y) {
                robot.airplane.release();
            }

            // update robot
            robot.update();
        }
    }
}
