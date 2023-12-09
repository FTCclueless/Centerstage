package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055Util;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Sensors {
    private LynxModule controlHub, expansionHub;
    private final HardwareQueue hardwareQueue;

    //private IMU imu;
    private int[] odometry = new int[] {0,0,0};
//    private final DigitalChannel magnetSensor;
    private final DigitalChannel intakeBeamBreak;
    private final DigitalChannel depositBeamBreak;

    private final AnalogInput analogInput0;
    private final AnalogInput analogInput1;
    private final AnalogInput analogInput2;
    private final AnalogInput analogInput3;

    private int slidesEncoder = 0;
    private double slidesVelocity = 0;
    private boolean slidesDown = false;
    private boolean intakeTriggered = false;
    private boolean depositTriggered = false;
    private double analog0Volt;
    private double analog1Volt;
    private double analog2Volt;
    private double analog3Volt;

    private BHI260IMU imu;
    private long imuLastUpdateTime = System.currentTimeMillis();
    private double imuHeading = 0.0;

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        this.hardwareQueue = hardwareQueue;

//        magnetSensor = hardwareMap.get(DigitalChannel.class, "magnetSensor");
        intakeBeamBreak = hardwareMap.get(DigitalChannel.class, "intakeBeamBreak");
        depositBeamBreak = hardwareMap.get(DigitalChannel.class, "depositBeamBreak");

        analogInput0 = hardwareMap.get(AnalogInput.class, "analogInput0");
        analogInput1 = hardwareMap.get(AnalogInput.class, "analogInput1");
        analogInput2 = hardwareMap.get(AnalogInput.class, "analogInput2");
        analogInput3 = hardwareMap.get(AnalogInput.class, "analogInput3");
        initHubs(hardwareMap);
    }

    private void initHubs(HardwareMap hardwareMap) {
        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            imu = hardwareMap.get(BHI260IMU.class, "imu");
            imu.initialize(
                    new IMU.Parameters(new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
                )
            );
            imu.resetYaw();
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    public void update() {
        updateControlHub();
        updateExpansionHub();
        updateTelemetry();
    }

    public double imuUpdateTime = 200;
    public double timeTillNextIMUUpdate = imuUpdateTime;
    public boolean imuJustUpdated = false;

    private void updateControlHub() {
        try {
            odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition(); // left (0)
            odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("rightRear")).motor[0].getCurrentPosition(); // right (3)
            odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition(); // back (1)

            if (System.currentTimeMillis() - imuLastUpdateTime >= imuUpdateTime) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                imuHeading = orientation.getYaw(AngleUnit.RADIANS);
                imuLastUpdateTime = System.currentTimeMillis();
                imuJustUpdated = true;
            } else {
                imuJustUpdated = false;
            }

            timeTillNextIMUUpdate = imuUpdateTime - (System.currentTimeMillis() - imuLastUpdateTime);

            intakeTriggered = intakeBeamBreak.getState();
            depositTriggered = depositBeamBreak.getState();
//            slidesDown = magnetSensor.getState();

            slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() * -1;
            slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getVelocity() * -1;
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    private void updateExpansionHub() {
        try {
            analog0Volt = analogInput0.getVoltage();
            analog1Volt = analogInput1.getVoltage();
            analog2Volt = analogInput2.getVoltage();
            analog3Volt = analogInput3.getVoltage();
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "expansion hub failed");
        }
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("imu heading (deg)", Math.toDegrees(getImuHeading()));
        TelemetryUtil.packet.put("Memory Usage", (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory())/1.0e6);
    }

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

    public double getImuHeading() {
        return imuHeading + Globals.START_HEADING_OFFSET;
    }

    public double getNormalizedIMUHeading() {
        return getImuHeading() - (numRotations*(2*Math.PI));
    }

    private double previousAngle = 0.0;
    private int numRotations = 0;
    private void addToCumulativeHeading(double angle) {
        if (Math.abs(angle-previousAngle) >= Math.toRadians(180)) {
            numRotations += Math.signum(previousAngle);
        }
        previousAngle = angle;
    }

    public double getAnalog0Volt() {
        return analog0Volt;
    }

    public double getAnalog1Volt() {
        return analog1Volt;
    }

    public double getAnalog2Volt() {
        return analog2Volt;
    }

    public double getAnalog3Volt() {
        return analog3Volt;
    }
}

