package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
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

    private IMU imu;
    private long imuLastUpdateTime = System.currentTimeMillis();
    private double imuHeading = 0.0;
    public boolean useIMU; // don't change the value here. Change in drivetrain.
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

        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        );
        imu.resetYaw();

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        depositLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        updateControlHub();
        updateExpansionHub();
        updateTelemetry();
    }

    private double imuUpdateTime = 200;
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

        imuJustUpdated = false;
        long currTime = System.currentTimeMillis();
        if (useIMU && currTime - imuLastUpdateTime >= imuUpdateTime) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            imuHeading = orientation.getYaw(AngleUnit.RADIANS);
            imuLastUpdateTime = currTime;
            imuJustUpdated = true;
        }

        timeTillNextIMUUpdate = imuUpdateTime - (currTime - imuLastUpdateTime);

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
        TelemetryUtil.packet.put("imu heading (deg)", Math.toDegrees(getImuHeading()));
        TelemetryUtil.packet.put("voltage", voltage);
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

    public double getImuHeading() {
        return imuHeading;
    }

    public double getVoltage() { return voltage; }

    public double getBackDist() { return backUltrasonicDist; }

    public double getFrontDist() { return frontUltrasonicDist; }

    public HuskyLens.Block[] getHuskyLensBlocks() {
        return huskyLensBlocks;
    }

    public double getNormalizedIMUHeading() {
        return getImuHeading() - (numRotations*(2*Math.PI));
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
    double ultrasonicDistThreshold = 125;

    boolean isBlocked = false;

    public void checkForPartner() {
        ultrasonicDist = getBackDist();
        Log.e("ultrasonicDist", ultrasonicDist + "");

        switch (ultrasonicCheckState) {
            case CHECK:
                Log.e("ultrasonic state", "CHECK");
                if (ultrasonicDist < ultrasonicDistThreshold) { // we are blocked
                    ultrasonicBlockedStart = System.currentTimeMillis();
                    ultrasonicDebounce = System.currentTimeMillis();
                    ultrasonicCheckState = UltrasonicCheckState.CONFIRM_BLOCKED;
                }
                break;
            case CONFIRM_BLOCKED:
                Log.e("ultrasonic state", "CONFIRM_BLOCKED");
                if (ultrasonicDist < ultrasonicDistThreshold) { // we are blocked
                    ultrasonicDebounce = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - ultrasonicDebounce > 75) { // if we detect that we are not blocked for 75 ms we assume false detection and go back
                    ultrasonicCheckState = UltrasonicCheckState.CHECK;
                }
                if (ultrasonicDebounce - ultrasonicBlockedStart > 125) { // we need to detect we are blocked for more than 100 ms with no breaks
                    startWaitTime = System.currentTimeMillis();
                    ultrasonicCheckState = UltrasonicCheckState.WAIT;
                }
                break;
            case WAIT:
                while (System.currentTimeMillis() - startWaitTime < blockedWaitTime) {
                    Log.e("ultrasonic state", "WAIT");
                    ultrasonicDist = getBackDist();
                    Log.e("ultrasonicDist", ultrasonicDist + "");
                }

                ultrasonicCheckState = UltrasonicCheckState.ALREADY_BLOCKED_IDLE;
                break;
            case ALREADY_BLOCKED_IDLE:
                Log.e("ultrasonic state", "ALREADY_BLOCKED_IDLE");
                break;
        }
    }
}

