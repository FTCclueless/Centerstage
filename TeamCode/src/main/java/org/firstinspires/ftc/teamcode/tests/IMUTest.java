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
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class IMUTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Sensors sensors = new Sensors(hardwareMap, null);
        Globals.RUNMODE = RunMode.TELEOP;
        TelemetryUtil.setup();

        waitForStart();

        while (opModeIsActive()) {
            sensors.update();

            TelemetryUtil.packet.put("IMU ANGLE", Math.toDegrees(sensors.getNormalizedIMUHeading()));
            TelemetryUtil.sendTelemetry();
        }
    }
}
