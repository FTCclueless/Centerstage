package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap, telemetry, true, false, true);
        Robot robot = new Robot(hardwareMap, vision);
        Globals.RUNMODE = RunMode.TESTER;

        waitForStart();

        while(!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);
            robot.update();
        }
    }
}
