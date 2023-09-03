package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Sensors {
    LynxModule controlHub, expansionHub;
    private HardwareQueue hardwareQueue;

    //private IMU imu;
    private int[] odometry = new int[2];

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        this.hardwareQueue = hardwareQueue;

        initHubs(hardwareMap);

        /*imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS,
                0, 0, 0,
                0 // Apparently unused
            ))
        );
        imu.initialize(params);*/
    }

    private void initHubs(HardwareMap hardwareMap) {
        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    public void update() {
        updateControlHub();
        // TODO: if loop times are actually trash limit this
        updateExpansionHub();
        updateTelemetry();
    }

    private void updateControlHub() {
        try {
            odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("rightRear")).motor[0].getCurrentPosition();
            odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition();
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    private void updateExpansionHub() {
        try {
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "expansion hub failed");
        }
    }

    public void updateTelemetry() {}

    public int[] getOdometry() {
        return odometry;
    }

    /*public IMU getImu() {
        return imu;
    }

    public double getIMUAngle() {
        Orientation robotOrientation;

        robotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        return normalizeAngle(robotOrientation.firstAngle);
    }*/

    double normalizedHeading = 0.0;
    double lastImuHeading = 0.0;

    /*public double normalizeAngle(double imuHeading) {
        double deltaHeading = imuHeading - lastImuHeading;
        while (Math.abs(deltaHeading) > Math.PI){
            deltaHeading -= Math.PI * 2 * Math.signum(deltaHeading);
        }
        lastImuHeading = imuHeading;
        normalizedHeading += deltaHeading;
        return normalizedHeading;
    }*/
}

