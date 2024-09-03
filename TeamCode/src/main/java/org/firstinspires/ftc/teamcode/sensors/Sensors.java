package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Sensors {
    private LynxModule controlHub, expansionHub;
    private final HardwareQueue hardwareQueue;
    private final HardwareMap hardwareMap;
    private Robot robot;

    //private IMU imu;
    private int[] odometry = new int[] {0,0,0};

    private int slidesEncoder;
    private double slidesVelocity;
    private boolean slidesDown = false;
    private boolean intakeTriggered = false;
    private boolean depositTouched = false;

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    private final AnalogInput backUltrasonic, frontUltrasonic;
    private double backUltrasonicDist, frontUltrasonicDist = 0;
    public double[] analogVoltages = new double[analogEncoders.length];

    private double voltage;

    private DigitalChannel depositLimitSwitch;

    private SparkFunOTOS otos;
    private double otosHeading = 0;
    private long numOtosLoops = 0;
    private double otosIntegral = 0;
    private double lastOtosIntegral = 0;
    private SparkFunOTOS.Pose2D sparkPose = new SparkFunOTOS.Pose2D();

    HuskyLens.Block[] huskyLensBlocks;

    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;

    public static double voltageK = 0.3;

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = hardwareQueue;
        this.robot = robot;

        backUltrasonic = hardwareMap.get(AnalogInput.class, "backUltrasonic");
        frontUltrasonic = hardwareMap.get(AnalogInput.class, "frontUltrasonic");

        depositLimitSwitch = hardwareMap.get(DigitalChannel.class, "depositLimitSwitch");
        otos = hardwareMap.get(SparkFunOTOS.class, "sparkfunSensor");
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        otos.calibrateImu();
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D( -3.333,2.9375, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.010);
        otos.setAngularScalar(0.992);
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);



        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        depositLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        updateControlHub();
        updateExpansionHub();
        updateTelemetry();
    }

    public void setOtosHeading(double heading) {
        otos.setPosition(new SparkFunOTOS.Pose2D( -3.333,2.9375, heading));
        lastOtosIntegral = otosHeading = otosIntegral = heading;
    }

    private double imuUpdateTime = 15;
    public double timeTillNextIMUUpdate = imuUpdateTime;
    public boolean imuJustUpdated = false;

    private double voltageUpdateTime = 5000;
    long lastVoltageUpdatedTime = System.currentTimeMillis();

    private double huskyUpdateTime = 100;
    long lastHuskyLensUpdatedTime = System.currentTimeMillis();
    public boolean huskyJustUpdated = false;

    private void updateControlHub() {
        odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition(); // left (0)
        odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("rightRear")).motor[0].getCurrentPosition(); // right (3)
        odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition(); // back (1)

        long currTime = System.currentTimeMillis();

        double lastOtosHeading = otosHeading;
        otosHeading = otos.getHeading() * -1;
        lastOtosIntegral = otosIntegral;
        otosIntegral += AngleUnit.normalizeRadians(lastOtosHeading - otosHeading) * 0.998197;
        TelemetryUtil.packet.put("OTOSHeading", Math.toDegrees(otosHeading));
        TelemetryUtil.packet.put("OTOSIntegral", Math.toDegrees(otosIntegral));

        if (currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = currTime;
        }

        slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() * -1;
        slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getVelocity() * -1;

        depositTouched = !depositLimitSwitch.getState();

        backUltrasonicDist = backUltrasonic.getVoltage() / 3.2 * 1000;
        frontUltrasonicDist = frontUltrasonic.getVoltage() / 3.2 * 1000;
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

    public void updateTelemetry() {
        TelemetryUtil.packet.put("voltage", voltage);
        TelemetryUtil.packet.put("ultrasonicDist", getBackDist());
        TelemetryUtil.packet.put("Ultrasonic State", ultrasonicCheckState);
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

    public boolean isDepositTouched() {
        return depositTouched;
    }

    public double getVoltage() { return voltage; }

    public double getBackDist() { return backUltrasonicDist; }

    public double getFrontDist() { return frontUltrasonicDist; }

    public HuskyLens.Block[] getHuskyLensBlocks() {
        return huskyLensBlocks;
    }

    public double getLastOtosHeading() {
        return lastOtosIntegral;
    }

    public double getOtosHeading() {
        return otosIntegral;
    }

    public void updateDrivetrainMotorCurrents() {
        leftFrontMotorCurrent = robot.drivetrain.leftFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        leftRearMotorCurrent = robot.drivetrain.leftRear.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        rightRearMotorCurrent = robot.drivetrain.rightRear.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        rightFrontMotorCurrent = robot.drivetrain.rightFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);

        TelemetryUtil.packet.put("leftFrontMotorCurrent", leftFrontMotorCurrent);
        TelemetryUtil.packet.put("leftRearMotorCurrent", leftRearMotorCurrent);
        TelemetryUtil.packet.put("rightRearMotorCurrent", rightRearMotorCurrent);
        TelemetryUtil.packet.put("rightFrontMotorCurrent", rightFrontMotorCurrent);
    }

    private double previousAngle = 0.0;
    private int numRotations = 0;
    private void addToCumulativeHeading(double angle) {
        if (Math.abs(angle-previousAngle) >= Math.toRadians(180)) {
            numRotations += Math.signum(previousAngle);
        }
        previousAngle = angle;
    }

    public enum UltrasonicCheckState {
        CHECK,
        CONFIRM_BLOCKED,
        WAIT,
        ALREADY_BLOCKED_IDLE
    }
    public UltrasonicCheckState ultrasonicCheckState = UltrasonicCheckState.CHECK;
    private long ultrasonicBlockedStart, startWaitTime;
    private long ultrasonicDebounce;
    double ultrasonicDist;

    double blockedWaitTime = 3500;
    double ultrasonicDistThreshold = 75;

    public void checkForPartner() {
        ultrasonicDist = getBackDist();
        Log.e("ultrasonicDist", ultrasonicDist + "");

        switch (ultrasonicCheckState) {
            case CHECK:
                if (ultrasonicDist < ultrasonicDistThreshold) { // we are blocked
                    ultrasonicBlockedStart = System.currentTimeMillis();
                    ultrasonicDebounce = System.currentTimeMillis();
                    ultrasonicCheckState = UltrasonicCheckState.CONFIRM_BLOCKED;
                }
                break;
            case CONFIRM_BLOCKED:
                if (ultrasonicDist < ultrasonicDistThreshold) { // we are blocked
                    ultrasonicDebounce = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - ultrasonicDebounce > 150) { // if we detect that we are not blocked for 75 ms we assume false detection and go back
                    ultrasonicCheckState = UltrasonicCheckState.CHECK;
                }
                if (ultrasonicDebounce - ultrasonicBlockedStart > 125) { // we need to detect we are blocked for more than 100 ms with no breaks
                    startWaitTime = System.currentTimeMillis();
                    ultrasonicCheckState = UltrasonicCheckState.WAIT;
                }
                break;
            case WAIT:
                if (System.currentTimeMillis() - startWaitTime > blockedWaitTime) {
                    ultrasonicCheckState = UltrasonicCheckState.ALREADY_BLOCKED_IDLE;
                }
                if (ultrasonicDist < ultrasonicDistThreshold) { // we are blocked
                    ultrasonicDebounce = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - ultrasonicDebounce > 250) { // if we detect that we are not blocked for 250 ms we assume partner is gone and leave
                    ultrasonicCheckState = UltrasonicCheckState.ALREADY_BLOCKED_IDLE;
                }
                break;
            case ALREADY_BLOCKED_IDLE:
                break;
        }
    }
}

