package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Disabled
@TeleOp
@Config
public class PriorityServoTest extends LinearOpMode {
    public static double angle = 0;
    public static double speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        PriorityServo testServo = new PriorityServo(
            hardwareMap.get(Servo.class, "testServo"),
            "testServo",
            PriorityServo.ServoType.SUPER_SPEED,
            1,
            0,
            1,
            0.5,
            true,
            1, 2
        );
        robot.hardwareQueue.addDevice(testServo);

        waitForStart();

        while (opModeIsActive()) {
            testServo.setTargetAngle(Math.toRadians(angle), speed);
            Globals.START_LOOP();
            robot.hardwareQueue.update();
        }
    }
}
