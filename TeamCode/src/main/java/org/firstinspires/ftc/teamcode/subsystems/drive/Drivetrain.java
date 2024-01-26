package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain {
    public enum State {
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

    public Localizer localizer;
    public Vision vision;

    public Drivetrain(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Vision vision) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;

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

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);

        if (vision != null) {
            if (vision.tagProcessor != null) {
                localizer = new Localizer(hardwareMap, sensors,true, true, vision, this);
                Log.e("using vision localizer", "");
            }
        } else {
            localizer = new Localizer(hardwareMap, sensors,false, true, null, this);
            Log.e("NOT using vision localizer", "");
        }
        setMinPowersToOvercomeFriction();
    }

    public Drivetrain (HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this(hardwareMap, hardwareQueue, sensors, null);
    }

    public void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeFriction(0.2933028305179981);
        leftRear.setMinimumPowerToOvercomeFriction(0.34301851543941514);
        rightRear.setMinimumPowerToOvercomeFriction(0.35201662929607813);
        rightFront.setMinimumPowerToOvercomeFriction(0.345230427428854);
//        leftFront.setMinimumPowerToOvercomeFriction(0.17493824636733907);
//        leftRear.setMinimumPowerToOvercomeFriction(0.2752498640527196);
//        rightRear.setMinimumPowerToOvercomeFriction(0.27855616387960147);
//        rightFront.setMinimumPowerToOvercomeFriction(0.22428003736833285);
//        leftFront.setMinimumPowerToOvercomeFriction(0.44669999999 * 0.7);
//        leftRear.setMinimumPowerToOvercomeFriction(0.4696999999999 * 0.7);
//        rightRear.setMinimumPowerToOvercomeFriction(0.474699999999999 * 0.7);
//        rightFront.setMinimumPowerToOvercomeFriction(0.42039999999997 * 0.7);
    }

    public void setLowerMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeFriction(0.2933028305179981/2);
        leftRear.setMinimumPowerToOvercomeFriction(0.34301851543941514/2);
        rightRear.setMinimumPowerToOvercomeFriction(0.35201662929607813/2);
        rightFront.setMinimumPowerToOvercomeFriction(0.345230427428854/2);
    }

    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeFriction(0.0);
        leftRear.setMinimumPowerToOvercomeFriction(0.0);
        rightRear.setMinimumPowerToOvercomeFriction(0.0);
        rightFront.setMinimumPowerToOvercomeFriction(0.0);
    }

    Pose2d targetPoint = new Pose2d(0,0,0);
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

    public static double turnBrakingAngleThreshold = 20; // in degrees
    public static double turnBrakingSpeedThreshold = 135; // in degrees
    public static double turnBrakingPower = -0.15;

    double targetForwardPower = 0;
    double targetStrafePower = 0;
    double targetTurnPower = 0;

    long perfectHeadingTimeStart = System.currentTimeMillis();

    Spline path = null;
    int pathIndex = 0;
    int pathRadius = 15;

    HuskyLens.Block[] huskyLensBlocks;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }
        updateLocalizer();
        Pose2d estimate = localizer.getPoseEstimate();
        ROBOT_POSITION = new Pose2d(estimate.x, estimate.y,estimate.heading);
        ROBOT_VELOCITY = localizer.getRelativePoseVelocity();

        if (path != null){
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
            targetPoint = pathTarget.clone();
            if (pathIndex == path.poses.size()){
                path = null;
                pathIndex = 0;
            } else {
                targetPoint.heading = Math.atan2(targetPoint.y - ROBOT_POSITION.y, targetPoint.x - ROBOT_POSITION.x);
            }
            targetPoint.heading += pathTarget.reversed ? Math.PI : 0;
        }

        calculateErrors();
        updateTelemetry();

        switch (state) {
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

                TelemetryUtil.packet.put("Stack Pose y", stackPose.y);
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
                state = State.WAIT_AT_POINT;
                break;
            case WAIT_AT_POINT:
                if (!atPoint()) {
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

    public void calculateErrors() {
        double deltaX = (targetPoint.x - localizer.x);
        double deltaY = (targetPoint.y - localizer.y);

        xError = Math.cos(localizer.heading)*deltaX + Math.sin(localizer.heading)*deltaY;
        yError = -Math.sin(localizer.heading)*deltaX + Math.cos(localizer.heading)*deltaY;
        turnError = targetPoint.heading-localizer.heading;

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

    public static PID xPID = new PID(0.085,0.0,0.01);
    public static PID yPID = new PID(0.1575,0.0,0.015);
    public static PID turnPID = new PID(0.5,0.0,0.01);

    public void PIDF() {
        double turnAdjustThreshold = (Math.abs(xError) > xThreshold/2 || Math.abs(yError) > yThreshold/2) ? turnThreshold/3.0 : turnThreshold;
        double fwd = Math.abs(xError) > xThreshold/2 ? xPID.update(xError, -maxPower, maxPower) + 0.05 * Math.signum(xError) : 0;
        double strafe = Math.abs(yError) > yThreshold/2 ? yPID.update(yError, -maxPower, maxPower) + 0.05 * Math.signum(yError) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(turnAdjustThreshold)/2? turnPID.update(turnError, -maxPower, maxPower) : 0;

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, turn);
    }

    public static PID finalXPID = new PID(0.1, 0.0,0.0);
    public static PID finalYPID = new PID(0.1, 0.0,0.0);
    public static PID finalTurnPID = new PID(0.075, 0.0,0.0);

    public void finalAdjustment() {
        double fwd = Math.abs(xError) > finalXThreshold/2 ? finalXPID.update(xError, -maxPower, maxPower) : 0;
        double strafe = Math.abs(yError) > finalYThreshold/2 ? finalYPID.update(yError, -maxPower, maxPower) : 0;
        double turn = Math.abs(turnError) > Math.toRadians(finalTurnThreshold)/2 ? finalTurnPID.update(turnError, -maxPower, maxPower) : 0;

        setLowerMinPowersToOvercomeFriction();

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
            motor.motor[0].setPower(0);
        }
    }

    public void updateLocalizer() {
        localizer.updateEncoders(sensors.getOdometry());
        localizer.update();
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("Drivetrain State", state);

        TelemetryUtil.packet.put("xError", xError);
        TelemetryUtil.packet.put("yError", yError);
        TelemetryUtil.packet.put("turnError (deg)", Math.toDegrees(turnError));

        TelemetryUtil.packet.fieldOverlay().setStroke("red");
        TelemetryUtil.packet.fieldOverlay().strokeCircle(targetPoint.x, targetPoint.y, xThreshold);
    }

    boolean finalAdjustment = false;
    boolean stop = true;
    double maxPower = 1.0;
    public void goToPoint(Pose2d targetPoint, boolean finalAdjustment, boolean stop, double maxPower) {
        this.finalAdjustment = finalAdjustment;
        this.stop = stop;
        this.maxPower = Math.abs(maxPower);

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

    public void setPath(Spline path) {
        this.path = path;
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
        leftFront.setTargetPower(lf);
        leftRear.setTargetPower(lr);
        rightRear.setTargetPower(rr);
        rightFront.setTargetPower(rf);
    }

    private void normalizeArray(double[] arr) {
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

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
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