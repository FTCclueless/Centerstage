package org.firstinspires.ftc.teamcode.vision.apriltags;


import android.util.Log;

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
import java.util.HashMap;
import java.util.Map;

public class AprilTagLocalizer {
    private AprilTagProcessor tagProcessor;

    private ArrayList<AprilTagDetection> tags = new ArrayList<AprilTagDetection>();

    public AprilTagLocalizer(Vision vision) {
        this.tagProcessor = vision.tagProcessor;
    }

    Pose2d cameraOffset = new Pose2d(-6.69, 0.0, Math.toRadians(180));

    double robotXFromTag = 0;
    double robotYFromTag = 0;

    Pose2d poseEstimate = new Pose2d(0,0,0);

    double closetDistance = 0.0;
    AprilTagDetection closestTag;

    public Pose2d update(double inputHeading) {
        try {
            if (tagProcessor.getDetections().size() > 0) {
                tags = tagProcessor.getDetections();

                closetDistance = getDistance(tags.get(0));
                closestTag = tags.get(0);

                for (AprilTagDetection tag : tags) {
                    double dist = getDistance(tag);
                    if (closetDistance == -1 || (dist != -1 && dist < closetDistance)) {
                        closetDistance = getDistance(tag);
                        closestTag = tag;
                    }
                }

                if (closetDistance == -1){
                    return null;
                }

                Vector3 globalTagPosition = convertVectorFToPose3d(closestTag.metadata.fieldPosition);

                Pose2d correctedTagData = new Pose2d(
                        closestTag.ftcPose.y * Math.cos(Math.toRadians(30)) + Math.cos(Math.toRadians(60)) * -closestTag.ftcPose.z,
                        -closestTag.ftcPose.x);

                Pose2d relativeTagPosition = new Pose2d(
                        correctedTagData.x * Math.cos(cameraOffset.heading) - correctedTagData.y * Math.sin(cameraOffset.heading) + cameraOffset.x,
                        correctedTagData.x * Math.sin(cameraOffset.heading) + correctedTagData.y * Math.cos(cameraOffset.heading) + cameraOffset.y);

                // applying a rotation matrix for converting from relative robot to global using the odo heading
                robotXFromTag = globalTagPosition.getX() - (Math.cos(inputHeading) * relativeTagPosition.x - Math.sin(inputHeading) * relativeTagPosition.y);
                robotYFromTag = globalTagPosition.getY() - (Math.sin(inputHeading) * relativeTagPosition.x + Math.cos(inputHeading) * relativeTagPosition.y);

                poseEstimate = new Pose2d(robotXFromTag, robotYFromTag, inputHeading);

                TelemetryUtil.packet.put("globalTagPosition.x", globalTagPosition.x);
                TelemetryUtil.packet.put("globalTagPosition.y", globalTagPosition.y);
                TelemetryUtil.packet.put("robotXFromTag", robotXFromTag);
                TelemetryUtil.packet.put("robotYFromTag", robotYFromTag);

                return poseEstimate;
            }
        } catch (Error e) {
            Log.e("---------VISION ERROR---------", e + "");
        }
        return null;
    }

    public double getDistance(AprilTagDetection tag) {
        if (tag.ftcPose != null && tag.metadata.fieldPosition.get(0) > 0) {
            return Math.sqrt(Math.pow(tag.ftcPose.y*Math.cos(Math.toRadians(30)) + Math.cos(Math.toRadians(60))*tag.ftcPose.z,2) + Math.pow(tag.ftcPose.x, 2));
        }
        return -1;
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

    public boolean detectedTag () {
       return tagProcessor.getDetections().size() > 0;
    }

    public static Vector3 convertVectorFToPose3d(VectorF vectorF) {
        return new Vector3(vectorF.get(0), vectorF.get(1), vectorF.get(2));
    }
}
