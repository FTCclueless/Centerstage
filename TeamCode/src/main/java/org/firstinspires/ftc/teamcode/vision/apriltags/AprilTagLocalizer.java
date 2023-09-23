package org.firstinspires.ftc.teamcode.vision.apriltags;

import android.util.Size;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class AprilTagLocalizer {
    HardwareMap hardwareMap;

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private ExposureControl exposure;
    private GainControl gain;

    private ArrayList<AprilTagDetection> tags = new ArrayList<AprilTagDetection>();
    private ArrayList<Integer> largeTags = new ArrayList<Integer>(Arrays.asList(7,10));

    public AprilTagLocalizer(HardwareMap hardwareMap) {
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

    double robotXFromTag = 0;
    double robotYFromTag = 0;

    private Pose2d robotPoseFromTag = new Pose2d(0, 0);
    private double averagedHeading = 0;

    MovingAverage movingAverage = new MovingAverage(1000);

    public void update() {
        if (tagProcessor.getDetections().size() > 0) {
            tags = tagProcessor.getDetections();

            for (AprilTagDetection tag : tags) {
                if (largeTags.contains(tag.id)) {
                    Pose2d globalTagPosition = convertVectorFToPose2d(tag.metadata.fieldPosition);
                    Pose2d relativeTagPosition = new Pose2d(tag.ftcPose.y, tag.ftcPose.x * -1, -Math.toRadians(tag.ftcPose.yaw+(globalTagPosition.getX() > 0 ? 0: 180))); //transform from april tag to relative robot transform

                    // averaging heading out
                    movingAverage.addData(relativeTagPosition.heading);
                    averagedHeading = movingAverage.getMovingAverageForNum();

                    // applying a rotation matrix for converting from relative robot to robot
                    robotXFromTag = globalTagPosition.getX() - (Math.cos(averagedHeading)*relativeTagPosition.x - Math.sin(averagedHeading)*relativeTagPosition.y);
                    robotYFromTag = globalTagPosition.getY() - (Math.sin(averagedHeading)*relativeTagPosition.x + Math.cos(averagedHeading)*relativeTagPosition.y);

                    robotPoseFromTag = new Pose2d(robotXFromTag, robotYFromTag, averagedHeading);

                    TelemetryUtil.packet.put("globalTagPosition_X", globalTagPosition.getX());
                    TelemetryUtil.packet.put("globalTagPosition_Y", globalTagPosition.getY());

                    TelemetryUtil.packet.put("tag.ftcPose.yaw", tag.ftcPose.yaw);
                    TelemetryUtil.packet.put("relativeTagPosition.getX()", relativeTagPosition.getX());
                    TelemetryUtil.packet.put("relativeTagPosition.getY()", relativeTagPosition.getY());
                    TelemetryUtil.packet.put("relativeTagPosition.getHeading()", Math.toDegrees(relativeTagPosition.getHeading()));
                    TelemetryUtil.packet.put("averagedHeading", Math.toDegrees(averagedHeading));

                    updateField();
                }

                TelemetryUtil.packet.put("largeTags.contains(tag.id)", largeTags.contains(tag.id));
            }
        }
        TelemetryUtil.packet.put("number of tags detected", tagProcessor.getDetections().size());
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("X: " + String.format("%.2f", tags.get(0).ftcPose.x) + " Y: " + String.format("%.2f", tags.get(0).ftcPose.y) + " Z: " + String.format("%.2f", tags.get(0).ftcPose.z));
        telemetry.addLine("Roll: " + String.format("%.2f", tags.get(0).ftcPose.roll) + " Pitch: " + String.format("%.2f", tags.get(0).ftcPose.pitch) + " Yaw: " + String.format("%.2f", tags.get(0).ftcPose.yaw));

        telemetry.addData("max gain", gain.getMaxGain());
        telemetry.addData("min gain", gain.getMinGain());

        telemetry.update();
    }

    public void updateField() {
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();

        DashboardUtil.drawRobot(fieldOverlay, robotPoseFromTag);
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

    public boolean detectedTag () {
       return tagProcessor.getDetections().size() > 0;
    }

    public static Pose2d convertVectorFToPose2d(VectorF vectorF) {
        return new Pose2d(vectorF.get(0), vectorF.get(1));
    }

    public Pose2d getPoseEstimate() {
        return robotPoseFromTag;
    }
}
