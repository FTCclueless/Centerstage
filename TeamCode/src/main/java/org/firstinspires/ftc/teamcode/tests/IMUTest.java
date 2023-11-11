package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class IMUTest extends LinearOpMode {
    private long lastLoop = System.currentTimeMillis();
    private double angle = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        ));
        imu.initialize(params);
        imu.resetYaw();

        waitForStart();
        lastLoop = System.currentTimeMillis();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            if (System.currentTimeMillis() - lastLoop > 350) {
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                angle = angles.getYaw(AngleUnit.DEGREES);
                lastLoop = System.currentTimeMillis();
            }
            packet.put("IMU ANGLE", angle);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
