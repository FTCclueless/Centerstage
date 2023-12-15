package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.EndAffector;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp(name = "Axon MINI+ Tester")
@Config
public class EndAffectorTester extends LinearOpMode {
    public static double v4ServoAngle = 0;
    public static double botTurretAngle = 0;
    public static double topTurretAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        EndAffector ea = robot.deposit.endAffector;

        waitForStart();

        while (opModeIsActive()) {
            ea.v4Servo.setTargetAngle(Math.toRadians(v4ServoAngle),0.1);
            ea.botTurret.setTargetAngle(Math.toRadians(botTurretAngle),0.1);
            ea.topTurret.setTargetAngle(Math.toRadians(topTurretAngle),0.1);

            TelemetryUtil.packet.put("v4Bar", ea.v4Servo.inPosition() + " " + ea.v4Servo.getTargetAngle());
            TelemetryUtil.packet.put("bot", ea.botTurret.inPosition() + " " + ea.botTurret.getTargetAngle());
            TelemetryUtil.packet.put("top", ea.topTurret.inPosition() + " " + ea.topTurret.getTargetAngle());
            robot.update();
        }
    }
}