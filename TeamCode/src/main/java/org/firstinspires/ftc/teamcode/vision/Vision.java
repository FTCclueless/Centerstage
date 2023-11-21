package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {
    VisionPortal visionPortal;

    public Vision() {}

    public void initCamera(HardwareMap hardwareMap, VisionProcessor pipeline) {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(pipeline)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {} // waiting for camera to start streaming

        setCameraSettings();
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

    public void close () {
        visionPortal.close();
    }
}
