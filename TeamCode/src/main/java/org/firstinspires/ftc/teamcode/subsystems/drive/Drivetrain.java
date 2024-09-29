package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.BNOLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeSoloLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.OldLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.OneHundredMSIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain {

    public enum State {
        FOLLOW_SPLINE,
        GO_TO_POINT,
        DRIVE,
        FINAL_ADJUSTMENT,
        ALIGN_WITH_STACK,
        ALIGN_WITH_STACK_FINAL_ADJUSTMENT,
        BRAKE,
        WAIT_AT_POINT,
        IDLE
    }
    public State state = State.IDLE;

    public PriorityMotor leftFront, leftRear, rightRear, rightFront;
    private List<PriorityMotor> motors;

    private HardwareQueue hardwareQueue;
    private Sensors sensors;

    public Localizer[] localizers;
    public Vision vision;
    public Robot robot;

    public Drivetrain(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Vision vision, Robot robot) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.robot = robot;

        leftFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftFront"),
            "leftFront",
            3, 5, sensors
        );

        leftRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear",
            3, 5, sensors
        );
        rightRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear",
            3, 5, sensors
        );
        rightFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront",
            3, 5, sensors
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (PriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            hardwareQueue.addDevice(motor);
        }

        // resetting odos
        leftFront.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);

        localizers = new Localizer[]{
                new IMUMergeSoloLocalizer(hardwareMap, sensors, this, "#0000ff", "#ff00ff"),
                new IMULocalizer(hardwareMap, sensors, this, "#ff0000", "#00ff00"),
                new IMUMergeLocalizer(hardwareMap, sensors, this, "#ffff00", "#00ffff"),
                new OneHundredMSIMULocalizer(hardwareMap, sensors, this, "#aa0000", "#00ee00"),
                new Localizer(hardwareMap, sensors, this, "#0000aa", "#aa00aa")
        };
        setMinPowersToOvercomeFriction();
    }

    public void resetSlidesMotorRightFront() {
        Log.e("RESETTTING", "*****RESTETING RIGHT FRONT *************");

        rightFront.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Drivetrain (HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this(hardwareMap, hardwareQueue, sensors, null, robot);
    }

    // leftFront, leftRear, rightRear, rightFront
    double[] minPowersToOvercomeFriction = new double[] {
        0.3121803239920063,
        0.3533249418072871,
        0.36038420175052865,
        0.39695077434023707
    };

    public void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0]);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1]);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2]);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3]);
        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195);
        }
    }

    /*public void setLowerMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0]/2);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1]/2);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2]/2);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3]/2);
    }*/

    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        leftRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0);
        }
    }

    public Pose2d targetPoint = new Pose2d(0,0,0);
    Pose2d lastTargetPoint = new Pose2d(0,0,0);

    double xError = 0.0;
    double yError = 0.0;
    double turnError = 0.0;

    public static double xSlowdown = 20;
    public static double ySlowdown = 20;
    public static double turnSlowdown = 60;

    public static double kAccelX = 0.0;
    public static double kAccelY = 0.0;
    public static double kAccelTurn = 0.0;

    public static double xBrakingDistanceThreshold = 5;
    public static double xBrakingSpeedThreshold = 20;
    public static double xBrakingPower = -0.22;

    public static double yBrakingDistanceThreshold = 5;
    public static double yBrakingSpeedThreshold = 16;
    public static double yBrakingPower = -0.1;

    public static double centripetalTune = 0.5;
    public static double finalPIDThreshold = 9;
    public static double slowdownPoints = 3;
    public static double strafeTune = 0.15;
    boolean slowDown = false;

    public static double turnBrakingAngleThreshold = 20; // in degrees
    public static double turnBrakingSpeedThreshold = 135; // in degrees
    public static double turnBrakingPower = -0.15;

    double targetForwardPower = 0;
    double targetStrafePower = 0;
    double targetTurnPower = 0;

    long perfectHeadingTimeStart = System.currentTimeMillis();

    public boolean useUltrasonicDetection = false;

    Spline path = null;
    int pathIndex = 0;
    public static int pathRadius = 20;

    HuskyLens.Block[] huskyLensBlocks;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }
        updateLocalizer();
        Pose2d estimate = localizers[0].getPoseEstimate();
        ROBOT_POSITION = new Pose2d(estimate.x, estimate.y,estimate.heading);
        ROBOT_VELOCITY = localizers[0].getRelativePoseVelocity();


        if (path != null) {
            pathIndex = Math.min(path.poses.size()-1, pathIndex);
            state = State.FOLLOW_SPLINE;
            maxPower = path.poses.get(pathIndex).power;
            double lastRadius = path.poses.get(Math.max(0,pathIndex-1)).getDistanceFromPoint(estimate);
            double radiusToPath = path.poses.get(pathIndex).getDistanceFromPoint(estimate);
            while (radiusToPath < pathRadius && pathIndex != path.poses.size()) {
                radiusToPath = path.poses.get(pathIndex).getDistanceFromPoint(estimate);
                if (lastRadius > radiusToPath && radiusToPath > pathRadius/3.0){
                    break;
                }
                lastRadius = radiusToPath;
                pathIndex ++;
            }
            SplinePose2d pathTarget = path.poses.get(Math.min(path.poses.size()-1,pathIndex));
            lastTargetPoint = targetPoint;
            targetPoint = pathTarget.clone();
            if (path.poses.size() - 1 - pathIndex < slowdownPoints) {
                slowDown = true;
            }
            if (pathIndex == path.poses.size() && path.poses.get(path.poses.size()-1).getDistanceFromPoint(estimate) < finalPIDThreshold){
                state = State.GO_TO_POINT;
                maxPower/=2;
                path = null;
                pathIndex = 0;
            } else {
                targetPoint.heading = Math.atan2(targetPoint.y - ROBOT_POSITION.y, targetPoint.x - ROBOT_POSITION.x);
            }
            targetPoint.heading += pathTarget.reversed ? Math.PI : 0;
        }

        calculateErrors();
        updateTelemetry();

        if (useUltrasonicDetection) {
            updateBackUltrasonicDetection();
        }

        switch (state) {
            case FOLLOW_SPLINE:
                double radius = (xError*xError + yError*yError) / (2*yError);
                Log.e("radius", ""+radius);
                if (Math.abs(radius) < 50) {
                    Canvas canvas = TelemetryUtil.packet.fieldOverlay();
                    //canvas.setStroke("green");
                    canvas.strokeCircle(estimate.x - radius * Math.sin(estimate.heading), estimate.y + radius * Math.cos(estimate.heading), Math.abs(radius));
                }

                double speed = 0.25 + 0.75*Math.min(Math.abs(radius), 200)/200.0;

                double targetFwd = maxPower*speed*Math.signum(xError);
                if (slowDown) {
                    targetFwd *= 0.3;
                }
                double targetTurn = ((3.88193 * Math.exp(-3.94484 * Math.abs(targetFwd)) + 0.725107) * (TRACK_WIDTH)/ radius) * targetFwd;

                double centripetal = centripetalTune*targetFwd*targetFwd/radius;

                double lastDist = estimate.getDistanceFromPoint(targetPoint);
                int index = Math.max(pathIndex-1,0);
                double dist = estimate.getDistanceFromPoint(path.poses.get(index));
                while (dist < lastDist && index > 0) {
                    index--;
                    lastDist = dist;
                    dist = estimate.getDistanceFromPoint(path.poses.get(index));
                }

                Pose2d point = path.poses.get(index);
                double erX = point.x-estimate.x;
                double erY = point.y-estimate.y;
                double relY = -Math.sin(estimate.heading)*erX + Math.cos(estimate.heading)*erY;


                double strafe = Math.abs(relY) > 2 ? relY*strafeTune : 0;
                strafe = Math.max(Math.min(strafe, 0.2), -0.2);

                double fwd = targetFwd;
                double turn = targetTurn;
                double[] motorPowers = {
                        fwd - turn - centripetal - strafe,
                        fwd - turn + centripetal + strafe,
                        fwd + turn - centripetal - strafe,
                        fwd + turn + centripetal + strafe,
                };
                normalizeArray(motorPowers);

                setMotorPowers(motorPowers[0],motorPowers[1],motorPowers[2],motorPowers[3]);

                break;
            case GO_TO_POINT:
                setMinPowersToOvercomeFriction();
                PIDF();

                if (atPoint()) {
                    if (finalAdjustment) {
                        finalTurnPID.resetIntegral();
                        perfectHeadingTimeStart = System.currentTimeMillis();
                        state = State.FINAL_ADJUSTMENT;
                    } else {
                        state = State.BRAKE;
                    }
                }
                break;
            case FINAL_ADJUSTMENT:
                finalAdjustment();

                if (Math.abs(turnError) < Math.toRadians(finalTurnThreshold)) {
                    if (System.currentTimeMillis() - perfectHeadingTimeStart > 150) {
                        state = State.BRAKE;
                    }
                } else {
                    perfectHeadingTimeStart = System.currentTimeMillis();
                }
                break;
            case ALIGN_WITH_STACK:
                huskyLensBlocks = sensors.getHuskyLensBlocks();
                if (sensors.huskyJustUpdated && huskyLensBlocks != null) {
                    if (huskyLensBlocks.length > 0) {
                        updateStackLocation(huskyLensBlocks);
                    }
                }
                targetPoint = new Pose2d(stackPose.x + intakeOffset, stackPose.y, Math.PI);

                PIDF();

                if (atPointThresholds(1.0, 1.0, 5)) {
                    state = State.ALIGN_WITH_STACK_FINAL_ADJUSTMENT;
                }

                break;
            case ALIGN_WITH_STACK_FINAL_ADJUSTMENT:
                huskyLensBlocks = sensors.getHuskyLensBlocks();
                if (sensors.huskyJustUpdated && huskyLensBlocks != null) {
                    if (huskyLensBlocks.length > 0) {
                        updateStackLocation(huskyLensBlocks);
                    }
                }
                targetPoint = new Pose2d(stackPose.x + intakeOffset, stackPose.y, Math.PI);

                finalAdjustment();

                if (atPointThresholds(0.5, 0.5, 5)) {
                    state = State.BRAKE;
                }
                break;
            case BRAKE:
                stopAllMotors();
                slowDown = false;
                state = State.WAIT_AT_POINT;
                break;
            case WAIT_AT_POINT:
                if (!atPointThresholds(1.5, 1.5, 5)) {
                    resetIntegrals();
                    state = State.GO_TO_POINT;
                }
                break;
            case DRIVE:
                break;
            case IDLE:
                break;
        }
    }

    double ultrasonicDist = 0;
    double ultrasonicDistThreshold = 50;

    enum UltrasonicCheckState {
        CHECK,
        CONFIRM_BLOCKED,
        WAIT,
        ALREADY_BLOCKED_IDLE
    }
    private UltrasonicCheckState ultrasonicCheckState = UltrasonicCheckState.CHECK;
    private long ultrasonicBlockedStart, startWaitTime;
    private long ultrasonicDebounce;
    double blockedWaitTime = 3500;

    public void updateBackUltrasonicDetection() {
        updateUltrasonics();

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
                    updateUltrasonics();
                    forceStopAllMotors();
                }

                useUltrasonicDetection = false;
                ultrasonicCheckState = UltrasonicCheckState.ALREADY_BLOCKED_IDLE;
                break;
            case ALREADY_BLOCKED_IDLE:
                Log.e("ultrasonic state", "ALREADY_BLOCKED_IDLE");
                break;
        }
    }

    private void updateUltrasonics() {
        ultrasonicDist = sensors.getBackDist();
        Log.e("ultrasonicDist", ultrasonicDist + "");
    }

    public void calculateErrors() {
        double deltaX = (targetPoint.x - localizers[0].x);
        double deltaY = (targetPoint.y - localizers[0].y);

        xError = Math.cos(localizers[0].heading)*deltaX + Math.sin(localizers[0].heading)*deltaY;
        yError = -Math.sin(localizers[0].heading)*deltaX + Math.cos(localizers[0].heading)*deltaY;
        turnError = targetPoint.heading-localizers[0].heading;

        while(Math.abs(turnError) > Math.PI ){
            turnError -= Math.PI * 2 * Math.signum(turnError);
        }
    }

    public void feedforward() {
        targetForwardPower = Math.min(Math.pow(Math.abs(xError)/xSlowdown, 2),1.0)*Math.signum(xError);
        targetStrafePower = Math.min(Math.pow(Math.abs(yError)/ySlowdown, 0.5),1.0)*Math.signum(yError);
        targetTurnPower = Math.min(Math.pow(Math.abs(turnError)/Math.toRadians(turnSlowdown), 2),1.0)*Math.signum(turnError);

        double fwd = Math.abs(xError) > xThreshold/2 ? targetForwardPower + kAccelX*(targetForwardPower - ROBOT_VELOCITY.x/(Globals.MAX_X_SPEED*(12/sensors.getVoltage()))) : 0;
        double strafe = Math.abs(yError) > yThreshold/2 ? targetStrafePower + kAccelY*(targetStrafePower - ROBOT_VELOCITY.y/(Globals.MAX_Y_SPEED*(12/sensors.getVoltage()))) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(turnThreshold/2) ? targetTurnPower + kAccelTurn*(targetTurnPower - ROBOT_VELOCITY.heading/(Globals.MAX_HEADING_SPEED*(12/sensors.getVoltage()))) : 0;

        setMinPowersToOvercomeFriction();

        // electronic braking (turn off min power to overcome friction if we are braking)
        if (Math.abs(xError) < xBrakingDistanceThreshold && Math.abs(ROBOT_VELOCITY.x) > xBrakingSpeedThreshold) {
            fwd = xBrakingPower * Math.signum(xError);
            resetMinPowersToOvercomeFriction();
        }
//        if (Math.abs(yError) < yBrakingDistanceThreshold && Math.abs(ROBOT_VELOCITY.y) > yBrakingSpeedThreshold) {
//            strafe = yBrakingPower * Math.signum(yError);
//            resetMinPowersToOvercomeFriction();
//        }
        if (Math.abs(turnError) < Math.toRadians(turnBrakingAngleThreshold) && Math.abs(ROBOT_VELOCITY.heading) > Math.toRadians(turnBrakingSpeedThreshold)) {
            turn = turnBrakingPower * Math.signum(turnError);
            resetMinPowersToOvercomeFriction();
        }

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, turn);
    }

    public static PID xPID = new PID(0.04,0.0,0.003);
    public static PID yPID = new PID(0.125,0.0,0.0175);
    public static PID turnPID = new PID(0.25,0.0,0.01);

    double fwd, strafe, turn, turnAdjustThreshold, finalTargetPointDistance;

    public void PIDF() {
        double globalExpectedXError = (targetPoint.x - localizers[0].expected.x);
        double globalExpectedYError = (targetPoint.y - localizers[0].expected.y);

        if (path != null) {
            finalTargetPointDistance = Math.abs(Utils.calculateDistanceBetweenPoints(localizers[0].getPoseEstimate(), finalTargetPoint));
        } else {
            finalTargetPointDistance = 0;
        }

        // converting from global to relative
        double relExpectedXError = globalExpectedXError*Math.cos(localizers[0].heading) + globalExpectedYError*Math.sin(localizers[0].heading);
        double relExpectedYError = globalExpectedYError*Math.cos(localizers[0].heading) - globalExpectedXError*Math.sin(localizers[0].heading);

        if (Math.abs(finalTargetPointDistance) < 20) { // if we are under threshold switch to predictive PID
            fwd = Math.abs(relExpectedXError) > xThreshold/2 ? xPID.update(relExpectedXError, -maxPower, maxPower) + 0.05 * Math.signum(relExpectedXError) : 0;
            strafe = Math.abs(relExpectedYError) > yThreshold/2 ? yPID.update(relExpectedYError, -maxPower, maxPower) + 0.05 * Math.signum(relExpectedYError) : 0;
        } else {
            fwd = Math.abs(xError) > xThreshold/2 ? xPID.update(xError, -maxPower, maxPower) + 0.05 * Math.signum(xError) : 0;
            strafe = Math.abs(yError) > yThreshold/2 ? yPID.update(yError, -maxPower, maxPower) + 0.05 * Math.signum(yError) : 0;
        }
        // turn does not have predictiveError
        turnAdjustThreshold = (Math.abs(xError) > xThreshold/2 || Math.abs(yError) > yThreshold/2) ? turnThreshold/3.0 : turnThreshold;
        turn = Math.abs(turnError) > Math.toRadians(turnAdjustThreshold)/2? turnPID.update(turnError, -maxPower, maxPower) : 0;

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, turn);

        // Logging
//        TelemetryUtil.packet.put("expectedXError", globalExpectedXError);
//        TelemetryUtil.packet.put("expectedYError", globalExpectedYError);
    }

    public static PID finalXPID = new PID(0.035, 0.0,0.0);
    public static PID finalYPID = new PID(0.1, 0.0,0.0);
    public static PID finalTurnPID = new PID(0.01, 0.0,0.0);

    public void finalAdjustment() {
        double fwd = Math.abs(xError) > finalXThreshold/2 ? finalXPID.update(xError, -maxPower, maxPower) : 0;
        double strafe = Math.abs(yError) > finalYThreshold/2 ? finalYPID.update(yError, -maxPower, maxPower) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(finalTurnThreshold)/2 ? finalTurnPID.update(turnError, -maxPower, maxPower) : 0;

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, turn);
    }

    public Pose2d stackPose = new Pose2d(-69.03, -11.75);
    double intakeOffset = 0.0;

    public void startStackAlignment(Pose2d stackPose, double intakeOffset, double maxPower) {
        this.stackPose = stackPose;
        this.intakeOffset = intakeOffset;
        this.maxPower = maxPower;
        state = State.ALIGN_WITH_STACK;
    }

    private void updateStackLocation(HuskyLens.Block[] blocks) {
        HuskyLens.Block biggerBlock = blocks[0];
        for (HuskyLens.Block block : blocks) {
            if (block.width*block.height > biggerBlock.width*biggerBlock.height) {
                biggerBlock = block;
            }
        }

        double pixelStackError = 160-biggerBlock.x;
        double inchesPerPixel = inchesPerPixel(biggerBlock.y);
        double inchesStackError = inchesPerPixel * pixelStackError;

        stackPose = new Pose2d(-69.03, ROBOT_POSITION.y-inchesStackError*Math.cos(ROBOT_POSITION.heading));
    }

    public double inchesPerPixel(double y) {
        return y*0.0004048471726 - 0.1019934877;
    }

    public boolean isBusy() {
        return state != State.WAIT_AT_POINT && state != State.IDLE;
    }

    public void stopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setTargetPower(0);
        }
    }

    public void forceStopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setPowerForced(0.0);
        }
    }

    public void updateLocalizer() {
        for (Localizer l : localizers) {
            l.updateEncoders(sensors.getOdometry());
            l.update();
        }
        //oldLocalizer.update();
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("Drivetrain State", state);

        TelemetryUtil.packet.put("xError", xError);
        TelemetryUtil.packet.put("yError", yError);
        TelemetryUtil.packet.put("turnError (deg)", Math.toDegrees(turnError));

//        TelemetryUtil.packet.put("maxPower", maxPower);

        TelemetryUtil.packet.fieldOverlay().setStroke("red");
        TelemetryUtil.packet.fieldOverlay().strokeCircle(targetPoint.x, targetPoint.y, xThreshold);

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        if (path != null) {
            Pose2d last = path.poses.get(0);
            for (int i = 1; i < path.poses.size(); i++) {
                Pose2d next = path.poses.get(i);
                canvas.strokeLine(last.x, last.y, next.x, next.y);
                last = next;
            }
        }
    }

    boolean finalAdjustment = false;
    boolean stop = true;
    double maxPower = 1.0;
    public void goToPoint(Pose2d targetPoint, boolean finalAdjustment, boolean stop, double maxPower) {
        this.finalAdjustment = finalAdjustment;
        this.stop = stop;
        this.maxPower = Math.abs(maxPower);
        finalTargetPoint = targetPoint;

        if (targetPoint.x != lastTargetPoint.x || targetPoint.y != lastTargetPoint.y || targetPoint.heading != lastTargetPoint.heading) { // if we set a new target point we reset integral
            this.targetPoint = targetPoint;
            lastTargetPoint = targetPoint;

            resetIntegrals();

            state = State.GO_TO_POINT;
        }
    }

    public void setFinalAdjustment(boolean finalAdjustment) {
        this.finalAdjustment = finalAdjustment;
    }

    public void setStop(boolean stop) {
        this.stop = stop;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    public double getMaxPower() {return maxPower;}

    Pose2d finalTargetPoint;
    public void setPath(Spline path) {
        pathIndex = 0;
        this.path = path;

        if (path != null) {
            finalTargetPoint = path.poses.get(path.poses.size()-1);
        }
    }

    public Spline getPath() {
        return path;
    }

    public static double xThreshold = 1;
    public static double yThreshold = 1;
    public static double turnThreshold = 5;

    public static double finalXThreshold = 0.5;
    public static double finalYThreshold = 0.5;
    public static double finalTurnThreshold = 2.5;

    public void setBreakFollowingThresholds(Pose2d thresholds) {
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        turnThreshold = thresholds.getHeading();
    }

    public void resetIntegrals() {
        xPID.resetIntegral();
        yPID.resetIntegral();
        turnPID.resetIntegral();
        finalTurnPID.resetIntegral();
    }

    public boolean atPoint () {
        if (finalAdjustment && state != State.GO_TO_POINT) {
            return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < Math.toRadians(finalTurnThreshold);
        }
        if (!stop) {
            return Math.abs(xError) < xThreshold*3 && Math.abs(yError) < yThreshold*3 && Math.abs(turnError) < Math.toRadians(turnThreshold);
        }

        return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < Math.toRadians(turnThreshold);
    }

    public boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(turnError) < Math.toRadians(headingThresh);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setTargetPowerSmooth(lf);
        leftRear.setTargetPowerSmooth(lr);
        rightRear.setTargetPowerSmooth(rr);
        rightFront.setTargetPowerSmooth(rf);
    }

    public void normalizeArray(double[] arr) {
        double largest = 1;
        for (int i = 0; i < arr.length; i++) {
            largest = Math.max(largest, Math.abs(arr[i]));
        }
        for (int i = 0; i < arr.length; i++) {
            arr[i] /= largest;
        }
    }

    public void setMoveVector(Vector2 moveVector, double turn) {
        double[] powers = {
            moveVector.x - turn - moveVector.y,
            moveVector.x - turn + moveVector.y,
            moveVector.x + turn - moveVector.y,
            moveVector.x + turn + moveVector.y
        };
        normalizeArray(powers);
        if (slowDown && !(state == State.FINAL_ADJUSTMENT)) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] = powers[i]*0.3;
            }
        }
        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    public double smoothControls(double value) {
        return 0.5*Math.tan(1.12*value);
    }

    public void drive(Gamepad gamepad) {
        resetMinPowersToOvercomeFriction();
        state = State.DRIVE;

        double forward = smoothControls(gamepad.left_stick_y);
        double strafe = smoothControls(gamepad.left_stick_x);
        double turn = smoothControls(-gamepad.right_stick_x);

        Vector2 drive = new Vector2(forward,strafe);
        if (drive.mag() <= 0.05){
            drive.mul(0);
        }
        setMoveVector(drive,turn);
    }

    public static PID rotateTeleopPID = new PID(1.0,0.,0.01);

    public void rotate(Gamepad gamepad, double heading, double threshold, double maxPower) {
        if (heading != lastTargetPoint.heading) { // if we set a new target point we reset integral
            this.targetPoint = new Pose2d(localizers[0].x, localizers[0].y, Math.toRadians(heading));
            this.maxPower = Math.abs(maxPower);

            lastTargetPoint = targetPoint;

            resetIntegrals();
        }

        state = State.DRIVE;

        double forward = smoothControls(gamepad.left_stick_y);
        double strafe = smoothControls(gamepad.left_stick_x);
        double turn = Math.abs(turnError) > Math.toRadians(threshold)? rotateTeleopPID.update(turnError, -maxPower, maxPower) : 0;

        Log.e("turn", turn + "");

        Vector2 drive = new Vector2(forward,strafe);
        if (drive.mag() <= 0.05){
            drive.mul(0);
        }
        setMoveVector(drive,turn);
    }

    public Pose2d getPoseEstimate() {
        return localizers[0].getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose2d) {
        for (Localizer l : localizers) {
            l.setPoseEstimate(pose2d);
        }
        sensors.setOtosHeading(pose2d.heading);
    }

    public Vector2 lineCircleIntersection(Pose2d start, Pose2d end, Pose2d robot, double radius) {
        //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899

        Vector2 direction = new Vector2(end.x - start.x, end.y - start.y);
        Vector2 robot2start = new Vector2(start.x - robot.x, start.y - robot.y);

        double a = Vector2.dot(direction, direction);
        double b = 2 * Vector2.dot(robot2start, direction);

        double c = Vector2.dot(robot2start, robot2start) - radius * radius;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return null;
        } else {
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b + discriminant) / 2;
            double t2 = (-b - discriminant) / 2;

            if ((t1 >= 0) && (t1 <= 1)) {
                direction.mul(t1);
                return Vector2.add(direction, new Vector2(start.x, start.y));
            }
            if ((t2 >= 0) && (t2 <= 1)) {
                direction.mul(t2);
                return Vector2.add(direction, new Vector2(start.x, start.y));
            } else {
                return null;
            }
        }
    }
}