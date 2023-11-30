package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoBasicTest extends LinearOpMode {

    public static double servoSetPosition = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "V4BarServo2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            servo.setPosition(servoSetPosition);
        }
    }
}
