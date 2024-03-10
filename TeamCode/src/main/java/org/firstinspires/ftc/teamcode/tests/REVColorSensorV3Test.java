package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@TeleOp
public class REVColorSensorV3Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        REVColorSensorV3 colorSensorV3 = hardwareMap.get(REVColorSensorV3.class, "intakeColorSensor");
        REVColorSensorV3.ControlRequest controlRequest = new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.PROX_SENSOR_ENABLED);
        colorSensorV3.sendControlRequest(controlRequest);
        colorSensorV3.configurePS(REVColorSensorV3.PSResolution.EIGHT, REVColorSensorV3.PSMeasureRate.m6p25s);

        waitForStart();
        TelemetryUtil.setup();

        while (opModeIsActive()) {
            long start = System.currentTimeMillis();
            int val = colorSensorV3.readPS();
            //double val = colorSensorV3.getDistance(DistanceUnit.INCH);
            long end = System.currentTimeMillis();
            TelemetryUtil.packet.put("proximity", val);
            TelemetryUtil.packet.put("time", end - start);
            TelemetryUtil.sendTelemetry();
        }
    }
}
