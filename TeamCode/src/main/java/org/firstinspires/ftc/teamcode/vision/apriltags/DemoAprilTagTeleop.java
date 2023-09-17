package org.firstinspires.ftc.teamcode.vision.apriltags;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class DemoAprilTagTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(hardwareMap);
        Vision vision = new Vision(hardwareMap, telemetry);
        TelemetryUtil.setup();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            vision.updateLocalization(localizer);
            vision.updateField();
            TelemetryUtil.sendTelemetry();
        }
    }
}
