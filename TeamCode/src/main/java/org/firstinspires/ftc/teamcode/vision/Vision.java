package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

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

    public Vision (HardwareMap hardwareMap, Telemetry telemetry, boolean isRed, boolean initTeamProp, boolean initAprilTag) {
        if (initTeamProp && initAprilTag) {
            initTeamPropAndAprilTag(hardwareMap, telemetry, isRed);
        } else if (initAprilTag) {
            initAprilTag(hardwareMap);
        } else {
            initTeamProp(hardwareMap, telemetry, isRed);
        }
    }

    public void initTeamProp(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, isRed);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(teamPropDetectionPipeline)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.e("initializing camera....", "");
        } // waiting for camera to start streaming

        setCameraSettings();

        visionPortal.setProcessorEnabled(teamPropDetectionPipeline, true);
    }

    public void initAprilTag(HardwareMap hardwareMap) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(385.451, 385.451, 306.64, 240.025)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tagProcessor)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.e("initializing camera....", "");
        } // waiting for camera to start streaming

        setCameraSettings();

        visionPortal.setProcessorEnabled(tagProcessor, true);
    }

    public void initTeamPropAndAprilTag(HardwareMap hardwareMap, Telemetry telemetry, boolean isRed) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(385.451, 385.451, 306.64, 240.025)
                .build();

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, isRed);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(teamPropDetectionPipeline, tagProcessor)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Log.e("initializing camera....", "");
        } // waiting for camera to start streaming

        setCameraSettings();

        visionPortal.setProcessorEnabled(tagProcessor, true);
        visionPortal.setProcessorEnabled(teamPropDetectionPipeline, true);
    }

    public void initDualCamera(HardwareMap hardwareMap, VisionProcessor pipeline) {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(pipeline)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {} // waiting for camera to start streaming

        setCameraSettings();
    }

    public void setCameraSettings() {
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(0, TimeUnit.MILLISECONDS);

        Log.e("exposure supported", exposure.isExposureSupported() + "");

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(0);

        WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        whiteBalanceControl.setWhiteBalanceTemperature(whiteBalanceControl.getMinWhiteBalanceTemperature());

        Log.e("min white balance", whiteBalanceControl.getMinWhiteBalanceTemperature() + "");
    }

    public void start () {
        visionPortal.resumeStreaming();
    }

    public void stop () {
        visionPortal.stopStreaming();
    }

    public void enableAprilTag () {
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
