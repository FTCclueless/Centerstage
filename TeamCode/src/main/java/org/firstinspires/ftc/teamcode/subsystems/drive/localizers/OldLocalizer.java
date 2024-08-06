package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class OldLocalizer {
    /*public OldLocalizer(HardwareMap hardwareMap, Sensors sensors, boolean useAprilTag, boolean useIMU, Vision vision, Drivetrain drivetrain) {
        super(hardwareMap, sensors, useAprilTag, useIMU, vision, drivetrain);
    }

    @Override
    public void update() {

        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime-lastTime)/1.0E9;
        lastTime = currentTime;

        // Odometry

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        double relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);

        // constant accel
        Pose2d lastRelativePose = relHistory.get(0);

        double vrx = (relDeltaX + lastRelativePose.x)/2;
        double arx = 2*(relDeltaX-vrx);
        //v_x = vrx + arx*t
        double vry = (relDeltaY + lastRelativePose.y)/2;
        double ary = 2*(relDeltaY-vry);
        //v_y = vry + ary*t
        double vrh = (deltaHeading + lastRelativePose.heading)/2;
        double arh = (deltaHeading - vrh);
        //h = h1 + vry*t + ary*t^2

        AdaptiveQuadrature xQuadrature = new AdaptiveQuadrature(new double[] {vrx,arx},new double[] {heading,vrh,arh});
        AdaptiveQuadrature yQuadrature = new AdaptiveQuadrature(new double[] {vry,ary},new double[] {heading,vrh,arh});

        x += xQuadrature.evaluateCos(fidelity, 0, 1, 0) - yQuadrature.evaluateSin(fidelity, 0, 1, 0);
        y += yQuadrature.evaluateCos(fidelity, 0, 1, 0) + xQuadrature.evaluateSin(fidelity, 0, 1, 0);

        odoHeading = (encoders[1].getCurrentDist()-encoders[0].getCurrentDist())/(leftY-rightY);

        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));

        // IMU

        if (this.sensors.useIMU) {
            updateHeadingWithIMU(sensors.getImuHeading());
        }

        // April Tag Heading

        aprilTagHeadingMerge = 0;
        if (useAprilTag && nanoTimes.size() > 5) {
            aprilTagPose = aprilTagLocalizer.update(); // update april tags

            if (aprilTagPose != null && !aprilTagPose.isNaN()) {
                Pose2d errorBetweenInterpolatedPastPoseAndAprilTag = new Pose2d(
                        aprilTagPose.x - interpolatedPastPose.x,
                        aprilTagPose.y - interpolatedPastPose.y,
                        Utils.headingClip(aprilTagPose.heading - interpolatedPastPose.heading - headingDif)
                );

                maxVel = Math.sqrt(Math.pow(relCurrentVel.x,2) + Math.pow(relCurrentVel.y,2));
                // TODO: Tune weights
                weight = 4/Utils.minMaxClip(maxVel,8,40); // as speed increases we should decrease weight of april tags

                // resetting odo with april tag data
                Pose2d changeInPosition = new Pose2d(0,0,0);
                if (maxVel < 45) {
                    changeInPosition.x = errorBetweenInterpolatedPastPoseAndAprilTag.x * weight;
                    changeInPosition.y = errorBetweenInterpolatedPastPoseAndAprilTag.y * weight;
                }
                if (maxVel < 15 && Math.abs(relCurrentVel.heading) < Math.toRadians(180)) {
                    changeInPosition.heading = errorBetweenInterpolatedPastPoseAndAprilTag.heading * weight;
                }
                for (int i = 0; i < poseHistory.size(); i++){
                    poseHistory.get(i).add(changeInPosition);
                }

//                TelemetryUtil.packet.put("changeInPosition.x", changeInPosition.x);
//                TelemetryUtil.packet.put("changeInPosition.y", changeInPosition.y);
//                TelemetryUtil.packet.put("changeInPosition.heading", Math.toDegrees(changeInPosition.heading));

                Log.e("changeInPosition.x", changeInPosition.x + "");
                Log.e("changeInPosition.y", changeInPosition.y + "");
                Log.e("changeInPosition.heading", Math.toDegrees(changeInPosition.heading) + "");

                x += changeInPosition.x;
                y += changeInPosition.y;
                aprilTagHeadingMerge = changeInPosition.heading;
            }
        }

//        mergeUltrasonics();

        heading = odoHeading + imuMerge + headingOffset + aprilTagHeadingMerge + Globals.START_HEADING_OFFSET;

        currentPose = new Pose2d(x, y, heading);

        nanoTimes.add(0, System.nanoTime());
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public void updateField() {
        TelemetryUtil.packet.put("x old", x);
        TelemetryUtil.packet.put("y old", y);
        TelemetryUtil.packet.put("heading (deg) old", Math.toDegrees(heading));

//        TelemetryUtil.packet.put("x speed", relCurrentVel.x);
//        TelemetryUtil.packet.put("y speed", relCurrentVel.y);
//        TelemetryUtil.packet.put("turn speed (deg)", Math.toDegrees(relCurrentVel.heading));

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate(), "#000000");
        DashboardUtil.drawRobot(fieldOverlay, expected, "#BB00BB");
    }*/
}
