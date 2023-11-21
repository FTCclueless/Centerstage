package org.firstinspires.ftc.teamcode.vision.apriltags;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagLocalizer {
    private AprilTagProcessor tagProcessor;
    private Vision vision = new Vision();

    private ArrayList<AprilTagDetection> tags = new ArrayList<AprilTagDetection>();
    private ArrayList<Integer> desiredTags = new ArrayList<Integer>(Arrays.asList(2,5,8,9));

    public AprilTagLocalizer(HardwareMap hardwareMap) {
        this.tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setLensIntrinsics(385.451, 385.451, 306.64, 240.025)
            .build();

       vision.initCamera(hardwareMap, tagProcessor);
    }

    Pose2d cameraOffset = new Pose2d(-7.36, 0.0, Math.toRadians(180));

    double robotXFromTag = 0;
    double robotYFromTag = 0;

    Pose2d poseEstimate = new Pose2d(0,0,0);

    public Pose2d update(double inputHeading) {
        if (tagProcessor.getDetections().size() > 0) {
            tags = tagProcessor.getDetections();
            for (AprilTagDetection tag : tags) {
                if (desiredTags.contains(tag.id)) {
                    Vector3 globalTagPosition = convertVectorFToPose3d(tag.metadata.fieldPosition);

                    Pose2d correctedTagData = new Pose2d(
                            tag.ftcPose.y*Math.cos(Math.toRadians(30)) + Math.cos(Math.toRadians(60))*tag.ftcPose.z,
                            tag.ftcPose.x);

                    Pose2d relativeTagPosition = new Pose2d(
                            correctedTagData.x*Math.cos(cameraOffset.heading) - correctedTagData.y*Math.sin(cameraOffset.heading) + cameraOffset.x,
                            correctedTagData.x*Math.sin(cameraOffset.heading) + correctedTagData.y*Math.cos(cameraOffset.heading) + cameraOffset.y);

                    TelemetryUtil.packet.put("correctedTagData.getX()", correctedTagData.getX());
                    TelemetryUtil.packet.put("correctedTagData.getY()", correctedTagData.getY());

                    TelemetryUtil.packet.put("relativeTagPosition.getX()", relativeTagPosition.getX());
                    TelemetryUtil.packet.put("relativeTagPosition.getY()", relativeTagPosition.getY());

//                    TelemetryUtil.packet.put("globalTagPosition.getX()", globalTagPosition.getX());
//                    TelemetryUtil.packet.put("globalTagPosition.getY()", globalTagPosition.getY());
//                    TelemetryUtil.packet.put("globalTagPosition.getZ()", globalTagPosition.getZ());

//                    TelemetryUtil.packet.put("correctedTagData.x", correctedTagData.x);
//                    TelemetryUtil.packet.put("correctedTagData.y", correctedTagData.y);
//                    TelemetryUtil.packet.put("tag.ftcPose.yaw", tag.ftcPose.yaw);
//                    TelemetryUtil.packet.put("tag.ftcPose.pitch", tag.ftcPose.pitch);
//                    TelemetryUtil.packet.put("tag.ftcPose.roll", tag.ftcPose.roll);

                    // applying a rotation matrix for converting from relative robot to global using the odo heading
                    robotXFromTag = globalTagPosition.getX() - (Math.cos(inputHeading) * relativeTagPosition.x - Math.sin(inputHeading) * relativeTagPosition.y);
                    robotYFromTag = globalTagPosition.getY() - (Math.sin(inputHeading) * relativeTagPosition.x + Math.cos(inputHeading) * relativeTagPosition.y);

                    TelemetryUtil.packet.put("robotXFromTag", robotXFromTag);
                    TelemetryUtil.packet.put("robotYFromTag", robotXFromTag);

                    poseEstimate = new Pose2d(robotXFromTag, robotYFromTag, inputHeading);

                    return poseEstimate;
                }
            }
        }
        return null;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("X: " + String.format("%.2f", tags.get(0).ftcPose.x) + " Y: " + String.format("%.2f", tags.get(0).ftcPose.y) + " Z: " + String.format("%.2f", tags.get(0).ftcPose.z));
        telemetry.addLine("Roll: " + String.format("%.2f", tags.get(0).ftcPose.roll) + " Pitch: " + String.format("%.2f", tags.get(0).ftcPose.pitch) + " Yaw: " + String.format("%.2f", tags.get(0).ftcPose.yaw));
        telemetry.update();
    }

    public void updateField() {
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate());
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void start () {
        vision.start();
    }

    public void stop () {
        vision.stop();
    }

    public void close () {
        vision.close();
    }

    public boolean detectedTag () {
       return tagProcessor.getDetections().size() > 0;
    }

    public static Vector3 convertVectorFToPose3d(VectorF vectorF) {
        return new Vector3(vectorF.get(0), vectorF.get(1), vectorF.get(2));
    }
}
