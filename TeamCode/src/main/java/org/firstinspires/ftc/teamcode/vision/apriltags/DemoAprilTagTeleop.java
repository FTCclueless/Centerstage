package org.firstinspires.ftc.teamcode.vision.apriltags;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class DemoAprilTagTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {} // waiting for camera to start streaming

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(40);

        long start = 0;
        long loopTime = 0;

        while (!isStopRequested() && opModeIsActive()) {
            start = System.currentTimeMillis();

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addLine("X: " + String.format("%.2f", tag.ftcPose.x) + " Y: " + String.format("%.2f", tag.ftcPose.y) + " Z: " + String.format("%.2f", tag.ftcPose.z));
                telemetry.addLine("Roll: " + String.format("%.2f", tag.ftcPose.roll) + " Pitch: " + String.format("%.2f", tag.ftcPose.pitch) + " Yaw: " + String.format("%.2f", tag.ftcPose.yaw));

                telemetry.addData("max gain", gain.getMaxGain());
                telemetry.addData("min gain", gain.getMinGain());
            }

            loopTime = System.currentTimeMillis() - start;
            telemetry.addData("loopTime", loopTime + "");

            telemetry.update();
        }
    }
}
