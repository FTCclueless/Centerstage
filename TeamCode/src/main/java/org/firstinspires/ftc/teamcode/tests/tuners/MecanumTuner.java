package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@Config
@TeleOp
public class MecanumTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        waitForStart();
        drivetrain.setMotorPowers(1,1,1,1);
        double start = System.currentTimeMillis();
        if (System.currentTimeMillis()  >= start + 1500) {
            drivetrain.setMotorPowers(0,0,0,0);
            TelemetryUtil.packet.put("heading", drivetrain.localizers[0].heading);
            TelemetryUtil.packet.put("x", drivetrain.localizers[0].x);
            TelemetryUtil.packet.put("y",drivetrain.localizers[0].y);
            drivetrain.update();
        }
    }
}
