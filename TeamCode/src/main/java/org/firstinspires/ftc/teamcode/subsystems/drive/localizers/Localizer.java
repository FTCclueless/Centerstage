package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.ArrayList;

public class Localizer {
    protected Sensors sensors;
    protected Drivetrain drivetrain;

    public Encoder[] encoders;
    protected long lastTime = System.nanoTime();

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    protected double odoHeading = 0;

    public Pose2d expected = new Pose2d(0, 0, 0);
    public Pose2d currentPose = new Pose2d(0,0,0);
    public Pose2d currentVel = new Pose2d(0,0,0);
    public Pose2d relCurrentVel = new Pose2d(0,0,0);
    public Pose2d relCurrentAcc = new Pose2d(0,0,0);
    public Pose2d currentPowerVector = new Pose2d(0,0,0);

    protected ConstantAccelMath constAccelMath = new ConstantAccelMath();

    protected ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    protected ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    protected ArrayList<Long> nanoTimes = new ArrayList<Long>();

    protected Pose2d aprilTagPose = new Pose2d(0,0,0);

    protected double maxVel = 0.0;
    protected double startHeadingOffset = 0;
    protected String color;
    protected String expectedColor;

    public Localizer(HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor) {
        this.sensors = sensors;
        this.drivetrain = drivetrain;
        this.color = color;
        this.expectedColor = expectedColor;

        encoders = new Encoder[3];

        encoders[0] = new Encoder(new Pose2d(0,7.233361867),  -1); // left
        encoders[1] = new Encoder(new Pose2d(0,-6.7290861),-1); // right
        encoders[2] = new Encoder(new Pose2d(-9.018771197, 0),  -1); // back (7.1660442092285175)

        relHistory.add(new Pose2d(0,0,0));
        poseHistory.add(new Pose2d(0,0,0));
        nanoTimes.add(Long.valueOf(0));
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }
    double headingOffset = 0;
    public void setPose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        headingOffset += h - this.heading;
        currentPose = new Pose2d(x, y, h);
    }

    public Pose2d getPoseEstimate() {
        return new Pose2d(currentPose.x, currentPose.y, currentPose.heading);
    }


    public void setPoseEstimate(Pose2d pose2d) {
        setPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        startHeadingOffset = pose2d.getHeading();
    }

    public Pose2d getRelativePoseVelocity() {
        return new Pose2d(relCurrentVel.x, relCurrentVel.y, relCurrentVel.heading);
    }

    double fidelity = 1E-8;

    public double relDeltaX;
    public double relDeltaY;
    public double distanceTraveled = 0;

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
        relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        // constant accel
        Pose2d relDelta = new Pose2d(relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime,relDelta,currentPose);

        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public Pose2d interpolatedPastPose;

    public void findPastInterpolatedPose(long poseTime) {
        int indexOfDesiredNanoTime = 0;

        for (long time : nanoTimes) {
            if (time > poseTime) {
                indexOfDesiredNanoTime++;
            } else {
                break;
            }
        }

        indexOfDesiredNanoTime = Math.min(indexOfDesiredNanoTime, nanoTimes.size()-1);

        Pose2d pastTimeRobotPose = poseHistory.get(indexOfDesiredNanoTime).clone();
        Pose2d pastTimeRobotPose2 = poseHistory.get(Math.max(0, indexOfDesiredNanoTime-1)).clone();

        if (indexOfDesiredNanoTime != 0) {
            Pose2d errorInPastPoses = new Pose2d(
                    pastTimeRobotPose2.x - pastTimeRobotPose.x,
                    pastTimeRobotPose2.y - pastTimeRobotPose.y,
                    Utils.headingClip(pastTimeRobotPose2.heading - pastTimeRobotPose.heading)
            );

            double timeWeight = (double) (poseTime - nanoTimes.get(indexOfDesiredNanoTime)) /
                    (double) (nanoTimes.get(Math.max(0, indexOfDesiredNanoTime - 1)) - nanoTimes.get(indexOfDesiredNanoTime));

            interpolatedPastPose = new Pose2d(
                    pastTimeRobotPose.x + errorInPastPoses.x * timeWeight,
                    pastTimeRobotPose.y + errorInPastPoses.y * timeWeight,
                    pastTimeRobotPose.heading + errorInPastPoses.heading * timeWeight
            );
        } else {
            interpolatedPastPose = pastTimeRobotPose;
        }
    }

    double headingDif = 0.0;
    boolean firstLoop = true;
    double lastImuHeading;
    double lastOdoHeading;

    double imuMerge = 0;

    public void updateHeadingWithIMU(double imuHeading) {
    }

    double leftDist = 0.0;
    double rightDist = 0.0;

    double leftXOffset = 5.5;
    double leftYOffset = 2.625;
    double rightXOffset = 5.5;
    double rightYOffset = -2.625;

    double lastLeftDist = 0.0;
    double lastRightDist = 0.0;

    /*public void mergeUltrasonics() {
        //leftDist = sensors.getDistLeft();
        //rightDist = sensors.getDistRight();

        double x_sign = Math.abs(Utils.headingClip(heading)) < Math.toRadians(90) ? 1 : -1;
        double y_sign = Math.signum(Utils.headingClip(heading));

        Pose2d relativeWallLocationLeft = new Pose2d(leftXOffset + leftDist, leftYOffset);
        Pose2d globalWallLocationLeft = new Pose2d(
                x + Math.cos(heading)*relativeWallLocationLeft.x - Math.sin(heading)*relativeWallLocationLeft.y,
                y + Math.sin(heading)*relativeWallLocationLeft.x + Math.cos(heading)*relativeWallLocationLeft.y
        );

        Pose2d relativeWallLocationRight = new Pose2d(rightXOffset + rightDist, rightYOffset);
        Pose2d globalWallLocationRight = new Pose2d(
                x + Math.cos(heading)*relativeWallLocationRight.x - Math.sin(heading)*relativeWallLocationRight.y,
                y + Math.sin(heading)*relativeWallLocationRight.x + Math.cos(heading)*relativeWallLocationRight.y
        );

        if (Globals.mergeUltrasonics && leftDist != lastLeftDist && rightDist != lastRightDist) { // actually merging localization
            if (Math.abs(globalWallLocationLeft.x - 70.5 * x_sign) < 4) {
                x += (70.5 * x_sign - globalWallLocationLeft.x) * 0.1;
            }
            if (Math.abs(globalWallLocationLeft.y - 70.5 * y_sign) < 4) {
                y += (70.5 * y_sign - globalWallLocationLeft.y) * 0.1;
            }

            if (Math.abs(globalWallLocationRight.x - 70.5 * x_sign) < 4) {
                x += (70.5 * x_sign - globalWallLocationRight.x) * 0.1;
            }
            if (Math.abs(globalWallLocationRight.y - 70.5 * y_sign) < 4) {
                y += (70.5 * y_sign - globalWallLocationRight.y) * 0.1;
            }
        }

        lastLeftDist = leftDist;
        lastRightDist = rightDist;

        TelemetryUtil.packet.fieldOverlay().setStroke("green");
        TelemetryUtil.packet.fieldOverlay().strokeCircle(globalWallLocationLeft.x, globalWallLocationLeft.y, 1);

        TelemetryUtil.packet.fieldOverlay().setStroke("blue");
        TelemetryUtil.packet.fieldOverlay().strokeCircle(globalWallLocationRight.x, globalWallLocationRight.y, 1);
    }*/

    public void updatePowerVector(double[] p){
        for (int i = 0; i < p.length; i ++){
            p[i] = Math.max(Math.min(p[i],1),-1);
        }
        double forward = (p[0] + p[1] + p[2] + p[3]) / 4;
        double left = (-p[0] + p[1] - p[2] + p[3]) / 4; //left power is less than 1 of forward power
        double turn = (-p[0] - p[1] + p[2] + p[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;
        long start = nanoTimes.size() != 0 ? nanoTimes.get(0) : 0;
        for (int i = 0; i < nanoTimes.size(); i++){
            totalTime = (double)(start - nanoTimes.get(i)) / 1.0E9;
            if (totalTime <= targetVelTimeEstimate){
                actualVelTime = totalTime;
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            }
        }
        if (actualVelTime != 0) {
            relCurrentVel = new Pose2d(
                    (relDeltaXTotal) / actualVelTime,
                    (relDeltaYTotal) / actualVelTime,
                    (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
            );

            currentVel = new Pose2d(
                    relCurrentVel.x*Math.cos(heading) - relCurrentVel.y*Math.sin(heading),
                    relCurrentVel.x*Math.sin(heading) + relCurrentVel.y*Math.cos(heading),
                    relCurrentVel.heading
            );
        }
        else {
            currentVel = new Pose2d(0, 0, 0);
            relCurrentVel = new Pose2d(0, 0, 0);
        }
        while (lastIndex + 1 < nanoTimes.size()){
            nanoTimes.remove(nanoTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }

    double a = 0.0045;
    double b = -0.0728;
    double c = 0.8062;
    double d = Math.sqrt(c/a);

    protected void updateExpected() {
        double totalVel = Math.sqrt(Math.pow(currentVel.x, 2) + Math.pow(currentVel.y, 2));
        double distance = getExpectedDistance(totalVel);

        if (totalVel <= d) {
            distance = totalVel*(getExpectedDistance(d)/d);
        }

        if (totalVel <= 5) {
            expected.x = x;
            expected.y = y;
            return;
        }

        expected.x = x + distance * currentVel.x/totalVel;
        expected.y = y + distance * currentVel.y/totalVel;
        expected.heading = heading;
    }

    private double getExpectedDistance (double x) {
        return a*Math.pow(x,2) + b*x + c;
    }

    protected MovingAverage movingAverage = new MovingAverage(100);

    public double combineHeadings(double headingError) {
        movingAverage.addData(headingError);
        double averageError = movingAverage.getMovingAverageForNum();
        movingAverage.updateValsRetroactively(averageError);
        return movingAverage.getMovingAverageForNum();
    }

    public void updateField() {
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" x", x);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" y", y);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" heading (deg)", Math.toDegrees(heading));
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" distance", distanceTraveled);

//        TelemetryUtil.packet.put("x speed", relCurrentVel.x);
//        TelemetryUtil.packet.put("y speed", relCurrentVel.y);
//        TelemetryUtil.packet.put("turn speed (deg)", Math.toDegrees(relCurrentVel.heading));

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate(), color);
        DashboardUtil.drawRobot(fieldOverlay, expected, expectedColor);
    }
}