package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.EndAffector;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@TeleOp(name = "Axon MINI+ Tester")
@Config
public class EndAffectorTester extends LinearOpMode {
    public static double v4ServoAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        EndAffector endAffector = robot.deposit.endAffector;

        waitForStart();

        while (opModeIsActive()) {
            endAffector.v4Servo.setTargetAngle(Math.toRadians(v4ServoAngle),0.1);

            TelemetryUtil.packet.put("v4Bar angle", endAffector.v4Servo.getTargetAngle());
            TelemetryUtil.packet.put("topServo angle", endAffector.topServo.getTargetAngle());
            robot.update();
        }
    }
}