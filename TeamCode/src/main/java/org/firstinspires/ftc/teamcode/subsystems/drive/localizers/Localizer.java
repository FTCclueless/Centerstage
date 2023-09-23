package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.vision.apriltags.AprilTagLocalizer;

import java.util.ArrayList;

@Config
public class Localizer {

    public Encoder[] encoders;
    long lastTime = System.nanoTime();

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    public double odoX = 0;
    public double odoY = 0;
    public double odoHeading = 0;

    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d currentVel = new Pose2d(0,0,0);
    Pose2d relCurrentVel = new Pose2d(0,0,0);
    Pose2d currentPowerVector = new Pose2d(0,0,0);

    ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    ArrayList<Double> loopTimes = new ArrayList<Double>();

    BNO055IMU imu;

    public boolean useAprilTag;

    AprilTagLocalizer aprilTagLocalizer;
    Pose2d aprilTagPose = new Pose2d(0,0,0);
    double aprilTagWeight = 0.2;
    double maxVel = 0.0;

    public Localizer(HardwareMap hardwareMap, boolean useAprilTag) {
        encoders = new Encoder[3];

        encoders[0] = new Encoder(new Pose2d(0,7.233659277778),  -1); // left (y = 7.6861797267140135)
        encoders[1] = new Encoder(new Pose2d(0,-6.10600173333),1); // right (y = -5.664117306820334)
        encoders[2] = new Encoder(new Pose2d(-3, 0),  -1); // back (x = -2.16505140605)

        this.useAprilTag = useAprilTag;

        if (useAprilTag) {
            aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        }
    }

    public void getIMU(BNO055IMU imu){
        this.imu = imu;
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }

    public void setPose(double x, double y, double h){
        this.odoX = x;
        this.odoY = y;
        this.odoHeading += h - this.odoHeading;
//        imuTimeStamp = System.currentTimeMillis();
//        lastPose = new MyPose2d(x,y,h);
//        lastImuHeading = imu.getAngularOrientation().firstAngle;
//        relIMUHistory.clear();
//        loopIMUTimes.clear();
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

    double weight;

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

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

        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));
//        relIMUHistory.add(0,new MyPose2d(relDeltaX,relDeltaY,deltaHeading));

        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 - (1.0 - Math.cos(deltaHeading)) * r2;
            relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
        }
        odoX += relDeltaX * Math.cos(odoHeading) - relDeltaY * Math.sin(odoHeading);
        odoY += relDeltaY * Math.cos(odoHeading) + relDeltaX * Math.sin(odoHeading);

        odoHeading += deltaHeading;

        if (useAprilTag) {
            aprilTagLocalizer.update(); // update april tags\

            if (aprilTagLocalizer.detectedTag()) {
                aprilTagPose = aprilTagLocalizer.getPoseEstimate();

                maxVel = Math.sqrt(Math.pow(relCurrentVel.x,2) + Math.pow(relCurrentVel.y,2));
                weight = Math.abs(Math.min(1/maxVel, aprilTagWeight));

                x = kalmanFilter(odoX, aprilTagPose.x, weight);
                y = kalmanFilter(odoY, aprilTagPose.y, weight);
                heading = kalmanFilter(odoHeading, aprilTagPose.heading, weight);
            }
        } else {
            x = odoX;
            y = odoY;
            heading = odoHeading;
        }

        currentPose = new Pose2d(x, y, heading);

        loopTimes.add(0,loopTime);
        poseHistory.add(0,currentPose);
        updateVelocity();
    }

    public void updatePowerVector(double[] p){
        for (int i = 0; i < p.length; i ++){
            p[i] = Math.max(Math.min(p[i],1),-1);
        }
        double forward = (p[0] + p[1] + p[2] + p[3]) / 4;
        double left = (-p[0] + p[1] - p[2] + p[3]) / 4; //left power is less than 1 of forward power
        double turn = (-p[0] - p[1] + p[2] + p[3]) / 4;
        currentPowerVector.x = forward * Math.cos(odoHeading) - left * Math.sin(odoHeading);
        currentPowerVector.y = left * Math.cos(odoHeading) + forward * Math.sin(odoHeading);
        currentPowerVector.heading = turn;
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;
        for (int i = 0; i < loopTimes.size(); i++){
            totalTime += loopTimes.get(i);
            if (totalTime <= targetVelTimeEstimate){
                actualVelTime += loopTimes.get(i);
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            }
        }
        currentVel = new Pose2d(
                (poseHistory.get(0).getX() - poseHistory.get(lastIndex).getX()) / actualVelTime,
                (poseHistory.get(0).getY() - poseHistory.get(lastIndex).getY()) / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
        );
        relCurrentVel = new Pose2d(
                (relDeltaXTotal) / actualVelTime,
                (relDeltaYTotal) / actualVelTime,
                (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
        );
        while (lastIndex + 1 < loopTimes.size()){
            loopTimes.remove(loopTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }

    public double kalmanFilter (double value1, double value2, double value2Weight) {
        return (value1 * (1.0-value2Weight)) + (value2 * value2Weight);
    }

//    MyPose2d lastPose = new MyPose2d(0,0,0);
//    long imuTimeStamp = System.currentTimeMillis();
//    ArrayList<MyPose2d> relIMUHistory = new ArrayList<MyPose2d>();
//    ArrayList<Double> loopIMUTimes = new ArrayList<Double>();
//    double lastImuHeading = 0;
//
//    public void updateHeadingIMU() {
//        if (System.currentTimeMillis() - imuTimeStamp >= 200){
//            double imuHeading = imu.getAngularOrientation().firstAngle;
//            double deltaHeading = (imuHeading - lastImuHeading) - (heading - lastPose.heading);
//            while (Math.abs(deltaHeading) > Math.PI){
//                deltaHeading -= Math.PI * 2 * Math.signum(deltaHeading);
//            }
//            double sumLoopTime = 0;
//            for (int i = 0; i < loopIMUTimes.size(); i ++){
//                sumLoopTime += loopIMUTimes.get(i);
//            }
//            double backX = encoders[2].x;
//            for (int i = 0; i < loopIMUTimes.size(); i ++){
//                double relDeltaHeading = (loopIMUTimes.get(i)/sumLoopTime) * deltaHeading;
//                relIMUHistory.get(i).y -= relDeltaHeading * backX;
//                relIMUHistory.get(i).heading += relDeltaHeading;
//
//                if (relIMUHistory.get(i).heading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
//                    double r1 = relIMUHistory.get(i).x / deltaHeading;
//                    double r2 = relIMUHistory.get(i).y / deltaHeading;
//                    relIMUHistory.get(i).x = Math.sin(relIMUHistory.get(i).heading) * r1 - (1.0 - Math.cos(relIMUHistory.get(i).heading)) * r2;
//                    relIMUHistory.get(i).y = (1.0 - Math.cos(relIMUHistory.get(i).heading)) * r1 + Math.sin(relIMUHistory.get(i).heading) * r2;
//                }
//                lastPose.x += relIMUHistory.get(i).x * Math.cos(lastPose.heading) - relIMUHistory.get(i).y * Math.sin(lastPose.heading);
//                lastPose.y += relIMUHistory.get(i).y * Math.cos(lastPose.heading) + relIMUHistory.get(i).x * Math.sin(lastPose.heading);
//
//                lastPose.heading += relIMUHistory.get(i).heading;
//            }
//            relIMUHistory.clear();
//            loopIMUTimes.clear();
//            imuTimeStamp = System.currentTimeMillis();
//            x = lastPose.x;
//            y = lastPose.y;
//            heading = lastPose.heading;
//            currentPose = new MyPose2d(x,y,heading);
//            lastImuHeading = imuHeading;
//        }
//    }
}
