package org.firstinspires.ftc.teamcode.vision.apriltags;


import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagLocalizer {
    private AprilTagProcessor tagProcessor;
    private Vision vision = new Vision();

    private ArrayList<AprilTagDetection> tags = new ArrayList<AprilTagDetection>();
    private ArrayList<Integer> largeTags = new ArrayList<Integer>(Arrays.asList(7,10));

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

    double robotXFromTag = 0;
    double robotYFromTag = 0;

    public Pose2d update(double odoHeading) {
        if (tagProcessor.getDetections().size() > 0) {
            tags = tagProcessor.getDetections();
            for (AprilTagDetection tag : tags) {
                if (largeTags.contains(tag.id)) {
                    Pose2d globalTagPosition = convertVectorFToPose2d(tag.metadata.fieldPosition);
                    Pose2d relativeTagPosition = new Pose2d(tag.ftcPose.y, tag.ftcPose.x * -1, -Math.toRadians(tag.ftcPose.yaw + (globalTagPosition.getX() > 0 ? 0 : 180))); //transform from april tag to relative robot transform

                    // applying a rotation matrix for converting from relative robot to global using the odo heading
                    robotXFromTag = globalTagPosition.getX() - (Math.cos(odoHeading) * relativeTagPosition.x - Math.sin(odoHeading) * relativeTagPosition.y);
                    robotYFromTag = globalTagPosition.getY() - (Math.sin(odoHeading) * relativeTagPosition.x + Math.cos(odoHeading) * relativeTagPosition.y);

                    TelemetryUtil.packet.put("relativeTagPosition.getX()", relativeTagPosition.getX());
                    TelemetryUtil.packet.put("relativeTagPosition.getY()", relativeTagPosition.getY());
                    TelemetryUtil.packet.put("relativeTagPosition.getHeading()", Math.toDegrees(relativeTagPosition.getHeading()));

                    return new Pose2d(robotXFromTag, robotYFromTag, relativeTagPosition.heading);
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

    public static Pose2d convertVectorFToPose2d(VectorF vectorF) {
        return new Pose2d(vectorF.get(0), vectorF.get(1));
    }
}
