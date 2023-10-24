package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class IntakeTester extends LinearOpMode {
    public static int numPixelsHigh = 1;
    public static boolean reverse = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Intake intake = new Intake(hardwareMap, robot.hardwareQueue, robot.sensors);
        waitForStart();

        while (!isStopRequested()) {
            intake.setActuationPixelHeight(numPixelsHigh);
            if (reverse) {
                intake.reverse();
            }

            robot.update();
            intake.update();
        }
    }
}
