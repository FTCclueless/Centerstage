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
    private int[] odometry = new int[3];

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        this.hardwareQueue = hardwareQueue;

        initHubs(hardwareMap);
    }

    private void initHubs(HardwareMap hardwareMap) {
        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

//            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
//            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    public void update() {
        updateControlHub();
        // TODO: if loop times are actually trash limit this
//        updateExpansionHub();
        updateTelemetry();
    }

    private void updateControlHub() {
        try {
            odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition();
            odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition();
            odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition();
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
}

