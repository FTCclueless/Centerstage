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

        ea.setPower(0.1);

        waitForStart();

        while (opModeIsActive()) {
            ea.setV4Bar(Math.toRadians(v4ServoAngle));
            ea.setBotTurret(Math.toRadians(botTurretAngle));
            ea.setTopTurret(Math.toRadians(topTurretAngle));

            TelemetryUtil.packet.put("v4Bar", ea.checkV4() + " " + ea.getTargetPitch());
            TelemetryUtil.packet.put("bot", ea.checkBottom() + " " + ea.getBottomAngle());
            TelemetryUtil.packet.put("top", ea.checkTopTurret() + " " + ea.getTopAngle());
            robot.update();
        }
    }
}