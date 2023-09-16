package org.firstinspires.ftc.teamcode.vision.apriltags;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Vision {
    HardwareMap hardwareMap;

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private ExposureControl exposure;
    private GainControl gain;

    private Telemetry telemetry;

    private ArrayList<AprilTagDetection> tags = new ArrayList<AprilTagDetection>();
    private Pose2d tagPoses[] = {
        new Pose2d(3,3), // tag 1
        new Pose2d(3,3) // tag 2
    };

    public Vision (HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        this.hardwareMap = hardwareMap;

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {} // waiting for camera to start streaming

        this.exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(6, TimeUnit.MILLISECONDS);

        this.gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(250);
    }

    public void updateLocalization(ThreeWheelLocalizer localizer) {
        if (tagProcessor.getDetections().size() > 0) {
            tags = tagProcessor.getDetections();

            for (AprilTagDetection tag : tags) {
                Pose2d fieldTagPosition = convertVectorFToPose2d(tag.metadata.fieldPosition);
                Pose2d relativeTagPosition = new Pose2d(tag.ftcPose.x, tag.ftcPose.y);

                
            }
        }
    }

    public void updateTelemetry() {
        telemetry.addLine("X: " + String.format("%.2f", tags.get(0).ftcPose.x) + " Y: " + String.format("%.2f", tags.get(0).ftcPose.y) + " Z: " + String.format("%.2f", tags.get(0).ftcPose.z));
        telemetry.addLine("Roll: " + String.format("%.2f", tags.get(0).ftcPose.roll) + " Pitch: " + String.format("%.2f", tags.get(0).ftcPose.pitch) + " Yaw: " + String.format("%.2f", tags.get(0).ftcPose.yaw));

        telemetry.addData("max gain", gain.getMaxGain());
        telemetry.addData("min gain", gain.getMinGain());
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

    public static Pose2d convertVectorFToPose2d(VectorF vectorF) {
        return new Pose2d(vectorF.get(0), vectorF.get(1));
    }
}
