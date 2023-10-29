package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class V4Tuner extends LinearOpMode {
    public static double pos = 0.4;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Servo servo = hardwareMap.get(Servo.class, "V4BarServo");
        servo.setPosition(pos); //do this for v4 bar and 2 turrets to find base position, then
    }
}