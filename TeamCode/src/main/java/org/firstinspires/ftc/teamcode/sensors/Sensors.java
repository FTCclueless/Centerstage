package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.bosch.BHI260IMU;
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
import org.firstinspires.ftc.teamcode.utils.Globals;
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
//    private final DigitalChannel magnetSensor;
    private final DigitalChannel intakeBeamBreak;
    private final DigitalChannel depositBeamBreak;
    private HuskyLens huskyLens;
    private final AnalogInput distLeft, distRight;
    private final DigitalChannel limitSwitch1, limitSwitch2;

    private int slidesEncoder;
    private double slidesVelocity;
    private boolean slidesDown = false;
    private boolean intakeTriggered = false;
    private boolean depositTriggered = false;
    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    public double[] analogVoltages = new double[analogEncoders.length];
    private double voltage;
    private double distLeftVal, distRightVal;
    private double intakeCurrent;

    private BHI260IMU imu;
    private long imuLastUpdateTime = System.currentTimeMillis();
    private double imuHeading = 0.0;
    HuskyLens.Block[] huskyLensBlocks;
    private long limitTime = System.currentTimeMillis();

    public static double voltageK = 0.3;

    public Sensors(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = hardwareQueue;
        this.robot = robot;

        intakeBeamBreak = hardwareMap.get(DigitalChannel.class, "intakeBeamBreak");
        depositBeamBreak = hardwareMap.get(DigitalChannel.class, "depositBeamBreak");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        distLeft = hardwareMap.get(AnalogInput.class, "distLeft");
        distRight = hardwareMap.get(AnalogInput.class, "distRight");

        limitSwitch1 = hardwareMap.get(DigitalChannel.class, "limitSwitch1");
        limitSwitch2 = hardwareMap.get(DigitalChannel.class, "limitSwitch2");

        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
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

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        } catch (Exception e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
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
//        try {
            odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition(); // left (0)
            odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("rightRear")).motor[0].getCurrentPosition(); // right (3)
            odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition(); // back (1)
            intakeCurrent = ((PriorityMotor) hardwareQueue.getDevice("intake")).motor[0].getCurrent(CurrentUnit.MILLIAMPS);

//            if (System.currentTimeMillis() - imuLastUpdateTime >= imuUpdateTime) {
//                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//                imuHeading = orientation.getYaw(AngleUnit.RADIANS);
//                imuLastUpdateTime = System.currentTimeMillis();
//                imuJustUpdated = true;
//            } else {
//                imuJustUpdated = false;
//            }
//
//            timeTillNextIMUUpdate = imuUpdateTime - (System.currentTimeMillis() - imuLastUpdateTime);

            if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
                voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
                lastVoltageUpdatedTime = System.currentTimeMillis();
            }

            intakeTriggered = intakeBeamBreak.getState();
            depositTriggered = depositBeamBreak.getState();

            slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() * -1;
            slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getVelocity() * -1;

            distLeftVal = (distLeft.getVoltage() / 3.2) * 1000;
            distRightVal = (distRight.getVoltage() / 3.2) * 1000;

            if (limitSwitch1.getState() || limitSwitch2.getState()) {
                limitTime = System.currentTimeMillis();
            }

            if (System.currentTimeMillis() - lastHuskyLensUpdatedTime > huskyUpdateTime && (robot.drivetrain.state == Drivetrain.State.ALIGN_WITH_STACK || robot.drivetrain.state == Drivetrain.State.ALIGN_WITH_STACK_FINAL_ADJUSTMENT)) {
                huskyLensBlocks = huskyLens.blocks();
                lastHuskyLensUpdatedTime = System.currentTimeMillis();
                huskyJustUpdated = true;
            } else {
                huskyJustUpdated = false;
            }
//        }
//        catch (Exception e) {
//            Log.e("******* Error due to ", e.getClass().getName());
//            e.printStackTrace();
//            Log.e("******* fail", "control hub failed");
//        }
    }

    private void updateExpansionHub() {
        try {
//            for (int i = 0; i < analogVoltages.length; i++) {
//                analogVoltages[i] = analogVoltages[i] * (1-voltageK) + analogEncoders[i].getVoltage()*voltageK;
//            }
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

    public boolean isDepositTriggered() {
        return depositTriggered;
    }

    public double getImuHeading() {
        return imuHeading + Globals.START_HEADING_OFFSET;
    }

    public double getVoltage() { return voltage; }

    public double getDistLeft() { return distLeftVal; }

    public double getDistRight() { return distRightVal; }

    public boolean getLimit() {
        return (System.currentTimeMillis() - limitTime < 100);
    }

    public HuskyLens.Block[] getHuskyLensBlocks() {
        return huskyLensBlocks;
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

    public double getIntakeCurrent() {
        return intakeCurrent;
    }
}

