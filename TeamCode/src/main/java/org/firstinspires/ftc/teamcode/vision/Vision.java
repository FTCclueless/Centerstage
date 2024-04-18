package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {
    VisionPortal visionPortal;
    public TeamPropDetectionPipeline teamPropDetectionPipeline;
    public AprilTagProcessor tagProcessor;

    int cameraWidth = 640;
    int cameraHeight = 360;

    public Vision (HardwareMap hardwareMap, Telemetry telemetry, boolean isRed, boolean initTeamProp, boolean initAprilTag) {
        if (initAprilTag) {
            Log.e("USING APRIL TAGS", "");
        } else {
            Log.e("NOT --- USING APRIL TAGS", "");
        }

        initAprilTagCrashProtection(hardwareMap, telemetry);

        if (initTeamProp && initAprilTag) {
            initTeamPropAndAprilTag(hardwareMap, telemetry, isRed);
        } else if (initAprilTag) {
            initAprilTag(hardwareMap);
        } else {
            initTeamProp(hardwareMap, telemetry, isRed);
        }

        telemetry.addData("Finished", "initializing");
        telemetry.update();
    }

    public void initAprilTagCrashProtection(HardwareMap hardwareMap, Telemetry telemetry) {
        initAprilTag(hardwareMap);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 2000) {
            telemetry.addData("Initializing", "please wait for crash protection to finish.");
            telemetry.update();
        }
        visionPortal.close();
    }

    public void initTeamProp(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, isRed);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(teamPropDetectionPipeline)
                .build();

        // waiting for camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.e("initializing camera....", "");
        }

        setCameraSettings(1,206); // setCameraSettings(9,255);

        visionPortal.setProcessorEnabled(teamPropDetectionPipeline, true);
    }

    public void initAprilTag(HardwareMap hardwareMap) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(tagProcessor)
                .build();

        // waiting for camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.e("initializing camera....", "");
        }

        setCameraSettings(2,255);

        visionPortal.setProcessorEnabled(tagProcessor, true);
    }

    public void initTeamPropAndAprilTag(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, isRed);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(teamPropDetectionPipeline, tagProcessor)
                .build();

        // waiting for camera to start streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.e("initializing camera....", "");
        }

        setCameraSettings(8,145); // setCameraSettings(9,255);

        visionPortal.setProcessorEnabled(tagProcessor, true);
        visionPortal.setProcessorEnabled(teamPropDetectionPipeline, true);
    }

    public void setCameraSettings(int exposureVal, int gainVal) {
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(exposureVal, TimeUnit.MILLISECONDS);

        Log.e("exposure supported", exposure.isExposureSupported() + "");

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(gainVal);
    }

    public void start () {
        visionPortal.resumeStreaming();
    }

    public void stop () {
        visionPortal.stopStreaming();
    }

    public void enableAprilTag () {
        setCameraSettings(2,255);
        visionPortal.setProcessorEnabled(tagProcessor, true);
    }

    public void disableAprilTag () {
        visionPortal.setProcessorEnabled(tagProcessor, false);
    }

    public void enableTeamProp () {
        visionPortal.setProcessorEnabled(teamPropDetectionPipeline, true);
    }

    public void disableTeamProp () {
        visionPortal.setProcessorEnabled(teamPropDetectionPipeline, false);
    }
}
