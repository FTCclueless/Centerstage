package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;

public class TwoWheelLocalizer {
    public Encoder[] encoders;
    long lastTime = System.nanoTime();
    private long imuLastUpdateTime = System.currentTimeMillis();
    public double x = 0;
    public double y = 0;
    public double heading = 0;

    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d currentVel = new Pose2d(0,0,0);
    public Pose2d relCurrentVel = new Pose2d(0,0,0);
    Pose2d currentPowerVector = new Pose2d(0,0,0);

    ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    ArrayList<Double> loopTimeHistory = new ArrayList<Double>();
    private final BHI260IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap) {
        encoders = new Encoder[2];

        encoders[0] = new Encoder(new Pose2d(0,-7.351173256),  1); // left
        encoders[1] = new Encoder(new Pose2d(0,7.00091155),-1); // right
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(params);
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++) {
            this.encoders[i].update(encoders[i]);
        }

        if (System.currentTimeMillis() - imuLastUpdateTime >= 350) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double errorH = AngleUtil.clipAngle(orientation.getYaw(AngleUnit.RADIANS) - currentPose.heading);
            currentPose.heading += errorH / 2;
            imuLastUpdateTime = System.currentTimeMillis();
        }
    }

    public void setPose(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.heading += h - this.heading;
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(currentPose.x, currentPose.y, currentPose.heading);
    }

    public void setPoseEstimate(Pose2d pose2d) {
        setPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public Pose2d getPoseVelocity() {
        return new Pose2d(currentVel.x, currentVel.y, currentVel.heading);
    }

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1.0e9;
        lastTime = currentTime;

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;

        // This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        // Tank drive has no relative delta Y
        double relDeltaY = 0.0;
        // This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);

        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));

        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined.
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 - (1.0 - Math.cos(deltaHeading)) * r2;
            relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
        }
        x += relDeltaX * Math.cos(heading) - relDeltaY * Math.sin(heading);
        y += relDeltaY * Math.cos(heading) + relDeltaX * Math.sin(heading);

        heading += deltaHeading;

        currentPose = new Pose2d(x, y, heading);

        loopTimeHistory.add(0,loopTime);
        poseHistory.add(0,currentPose);
        updateVelocity();
        updateTelemetry();
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("x", currentPose.getX());
        TelemetryUtil.packet.put("y", currentPose.getY());
        TelemetryUtil.packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));
    }

    // powers = [leftFront, leftBack, rightFront, rightBack]
    public void updatePowerVector(double[] powers) {
        for (int i = 0; i < powers.length; i ++){
            powers[i] = Math.max(Math.min(powers[i],1),-1);
        }
        double forward = (powers[0] + powers[1] + powers[2] + powers[3]) / 4;
        double left = (-powers[0] + powers[1] - powers[2] + powers[3]) / 4; // left power is less than 1 of forward power
        double turn = (-powers[0] - powers[1] + powers[2] + powers[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.1; // in seconds
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;

        // looks through past loop times until the last loop time that is under the targetVelTimeEstimate
        for (int i = 0; i < loopTimeHistory.size(); i++){
            totalTime += loopTimeHistory.get(i);
            if (totalTime <= targetVelTimeEstimate) {
                actualVelTime += loopTimeHistory.get(i);
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            } else {
                break;
            }
        }
        if (actualVelTime != 0) {
            double averageHeadingVel = (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime;

            double x = 0.7 * (double) relCurrentVel.x + 0.3 * (relDeltaXTotal) / actualVelTime;
            double y = 0.7 * (double) relCurrentVel.y + 0.3 * (relDeltaYTotal) / actualVelTime;
            double heading = 0.7 * relCurrentVel.heading + averageHeadingVel;
            // relative velocity (can't do final minus initial because relative has a heading component)
            relCurrentVel = new Pose2d(
                    x,
                    y,
                    heading
            );

            // global velocity
            currentVel = new Pose2d(
                    relCurrentVel.x * Math.cos(heading) - relCurrentVel.y * Math.sin(heading),
                    relCurrentVel.x * Math.sin(heading) + relCurrentVel.y * Math.cos(heading),
                    averageHeadingVel
            );
        }

        // clearing arrays
        while (lastIndex + 1 < loopTimeHistory.size()) {
            loopTimeHistory.remove(loopTimeHistory.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }
}
