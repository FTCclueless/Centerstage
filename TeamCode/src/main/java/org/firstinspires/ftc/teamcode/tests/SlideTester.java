package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;

@TeleOp
@Config
public class SlideTester extends LinearOpMode {
    public static double distance = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Slides slides = new Slides(hardwareMap, robot.hardwareQueue, robot.sensors);
        //Deposit deposit = robot.deposit;
        //Slides slides = deposit.slides;
        waitForStart();

        slides.setLength(distance);
    }
}
