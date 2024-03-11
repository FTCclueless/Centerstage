package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

import java.util.ArrayList;

@Config
@TeleOp(group = "Test")
public class MotorTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<PriorityMotor> motors = new ArrayList<>();

        // buttons for changing motor
        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();

        int motorSize = 0;
        int motorIndex = 0;
        double motorPower = 0.0;

        // getting number of motors we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof PriorityMotor) {
                motors.add((PriorityMotor) device);
                motorSize++;
            }
        }

        waitForStart();

        while (!isStopRequested()) {
            robot.drivetrain.resetMinPowersToOvercomeFriction();

            if (buttonY.isClicked(gamepad1.y)) {
                motors.get(motorIndex).setTargetPower(0.0);
                motorIndex++;
                motorPower = 0.0;
            }

            if (buttonA.isClicked(gamepad1.a)) {
                motors.get(motorIndex).setTargetPower(0.0);
                motorIndex--;
                motorPower = 0.0;
            }

            if (gamepad1.b) {
                motorPower += 0.01;
            }

            if (gamepad1.x) {
                motorPower -= 0.01;
            }

            motorPower = Utils.minMaxClip(motorPower, -1.0, 1.0);
            motorIndex = Math.abs(motorIndex) % motorSize;

            motors.get(motorIndex).setTargetPower(motorPower);

            robot.update();

            telemetry.addData("motor index", motorIndex);
            telemetry.addData("motor name", motors.get(motorIndex).name);
            telemetry.addData("motor power", motorPower);
            telemetry.addData("motor velocity", motors.get(motorIndex).getVelocity());
            telemetry.update();
        }
    }
}
