package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@TeleOp
public class ServoBasicTest extends LinearOpMode {

    public static double servoSetPosition = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "bottomTurret");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            servo.setPosition(servoSetPosition);
        }
    }
}
