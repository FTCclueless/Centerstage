package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@TeleOp
@Config
public class IntakeTester extends LinearOpMode {
    public static double actuationAngle = 0;
    public static boolean reverseIntake = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        TelemetryUtil.setup();

        waitForStart();

        while (!isStopRequested()) {
            robot.intake.actuation.setTargetAngle(Math.toRadians(actuationAngle), 1.0);
            if (reverseIntake) {
                robot.intake.reverse();
            }

            TelemetryUtil.packet.put("actuationPosition", robot.intake.actuation.getTargetPosition());
            robot.update();
        }
    }
}
