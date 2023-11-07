package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Sensors {
    private LynxModule controlHub, expansionHub;
    private final HardwareQueue hardwareQueue;

    //private IMU imu;
    private int[] odometry = new int[] {0,0,0};
//    private final DigitalChannel magnetSensor;
//    private final DigitalChannel intakeBeamBreak;
//    private final DigitalChannel depositBeamBreak;

    private int slidesEncoder = 0;
    private double slidesVelocity = 0;
    private boolean slidesDown = false;
    private boolean intakeTriggered = false;
    private boolean depositTriggered = false;

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        this.hardwareQueue = hardwareQueue;

//        magnetSensor = hardwareMap.get(DigitalChannel.class, "magnetSensor");
//        intakeBeamBreak = hardwareMap.get(DigitalChannel.class, "intakeBeamBreak");
//        depositBeamBreak = hardwareMap.get(DigitalChannel.class, "depositBeamBreak");

        initHubs(hardwareMap);
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
        updateExpansionHub();
        updateTelemetry();
    }

    private void updateControlHub() {
        try {
            if (Globals.RUNMODE != RunMode.TELEOP) {
                odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition(); // left (0)
                odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition(); // right (3)
                odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("rightRear")).motor[0].getCurrentPosition(); // back (1)
            }

            //slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor0")).motor[0].getCurrentPosition();
            //slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor0")).motor[0].getVelocity();
            //slidesDown = magnetSensor.getState();
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    private void updateExpansionHub() {
        try {
//            slidesDown = magnetSensor.getState();
//            intakeTriggered = intakeBeamBreak.getState();
//            depositTriggered = depositBeamBreak.getState();
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

    public int getSlidesPos() {
        return slidesEncoder;
    }
    public double getSlidesVelocity() {
        return slidesVelocity;
    }

    public boolean isSlidesDown() {
        return slidesDown;
    }

    public boolean isIntakeTriggered() {
        return intakeTriggered;
    }

    public boolean isDepositTriggered() {
        return depositTriggered;
    }
}

