package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
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
        DRIVE,
        BRAKE,
        IDLE
    }
    public State state = State.BRAKE;

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
            3, 5
        );

        leftRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear",
            3, 5
        );
        rightRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear",
            3, 5
        );
        rightFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront",
            3, 5
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
                localizer = new Localizer(hardwareMap, sensors,true, true, vision);
                Log.e("using vision localizer", "erfjwlefwef");
            }
        } else {
            localizer = new Localizer(hardwareMap, sensors,false, true, null);
            Log.e("NOT using vision localizer", "123124r1cs");
        }
        setMinPowersToOvercomeFriction();
    }

    public Drivetrain (HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this(hardwareMap, hardwareQueue, sensors, null);
    }

    public void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeFriction(0.44669999999 * 0.7);
        leftRear.setMinimumPowerToOvercomeFriction(0.4696999999999 * 0.7);
        rightRear.setMinimumPowerToOvercomeFriction(0.474699999999999 * 0.7);
        rightFront.setMinimumPowerToOvercomeFriction(0.42039999999997 * 0.7);
    }

    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeFriction(0.0);
        leftRear.setMinimumPowerToOvercomeFriction(0.0);
        rightRear.setMinimumPowerToOvercomeFriction(0.0);
        rightFront.setMinimumPowerToOvercomeFriction(0.0);
    }

    public static PID xPID = new PID(0.085,0.0,0.01);
    public static PID yPID = new PID(0.15,0.0,0.01);
    public static PID turnPID = new PID(0.73,0.0,0.001);

    Pose2d targetPoint = new Pose2d(0,0,0);
    Pose2d lastTargetPoint = new Pose2d(0,0,0);

    double xError = 0.0;
    double yError = 0.0;
    double turnError = 0.0;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        updateLocalizer();
        updateTelemetry();

        Pose2d estimate = localizer.getPoseEstimate();
        ROBOT_POSITION = new Pose2d(estimate.x, estimate.y,estimate.heading);
        ROBOT_VELOCITY = localizer.getPoseVelocity();

        switch (state) {
            case DRIVE:
                double deltaX = (targetPoint.x - localizer.x);
                double deltaY = (targetPoint.y-localizer.y);

                xError = Math.cos(localizer.heading)*deltaX + Math.sin(localizer.heading)*deltaY;
                yError = -Math.sin(localizer.heading)*deltaX + Math.cos(localizer.heading)*deltaY;
                turnError = targetPoint.heading-localizer.heading;

                while(Math.abs(turnError) > Math.PI ){
                    turnError -= Math.PI * 2 * Math.signum(turnError);
                }

                double fwd = xPID.update(Math.abs(xError) > xThreshold/2 ? xError : 0);
                double strafe = yPID.update(Math.abs(yError) > yThreshold/2 ? yError : 0);
                double turn = turnPID.update(Math.abs(turnError) > headingThreshold/2 ? turnError : 0);

                Vector2 move = new Vector2(fwd, strafe);
                setMoveVector(move, turn);

                if (atPoint()) {
                    state = State.BRAKE;
                }
                break;
            case BRAKE:
                stopAllMotors();
                state = State.IDLE;
                break;
            case IDLE:
                break;
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public void stopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setTargetPower(0);
        }
    }

    public void updateLocalizer() {
        localizer.updateEncoders(sensors.getOdometry());
        localizer.update();
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("Drivetrain State", state);
        TelemetryUtil.packet.put("drivetrain at point", atPoint());

        TelemetryUtil.packet.put("xError", xError);
        TelemetryUtil.packet.put("yError", yError);
        TelemetryUtil.packet.put("turnError (deg)", Math.toDegrees(turnError));
    }

    public void goToPoint(Pose2d targetPoint) {
        TelemetryUtil.packet.fieldOverlay().setStroke("red");
        TelemetryUtil.packet.fieldOverlay().strokeCircle(targetPoint.x, targetPoint.y, xThreshold);

        if (targetPoint.x != lastTargetPoint.x || targetPoint.y != lastTargetPoint.y || targetPoint.heading != lastTargetPoint.heading) { // if we set a new target point we reset integral
            xPID.resetIntegral();
            yPID.resetIntegral();
            turnPID.resetIntegral();

            this.targetPoint = targetPoint;
            lastTargetPoint = targetPoint;
            state = State.DRIVE;
            Log.e("resetting integrals", "");
        }
    }

    public static double xThreshold = 1;
    public static double yThreshold = 1;
    public static double headingThreshold = Math.toRadians(5);

    public void setBreakFollowingThresholds(Pose2d thresholds) {
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        headingThreshold = thresholds.getHeading();
    }

    public boolean atPoint () {
        return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < headingThreshold;
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

        TelemetryUtil.packet.put("leftFront", powers[0]);
        TelemetryUtil.packet.put("leftRear", powers[1]);
        TelemetryUtil.packet.put("rightRear", powers[2]);
        TelemetryUtil.packet.put("rightFront", powers[3]);

        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    public double smoothControls(double value) {
        return 0.5*Math.tan(1.12*value);
    }

    public void drive(Gamepad gamepad) {
        resetMinPowersToOvercomeFriction();

        state = State.DRIVE;

        double forward = smoothControls(gamepad.left_stick_y);
        double strafe = smoothControls(-gamepad.left_stick_x);
        double turn = smoothControls(gamepad.right_stick_x);

        double[] powers = {
            forward + turn + strafe,
            forward + turn - strafe,
            forward - turn + strafe,
            forward - turn - strafe
        };
        normalizeArray(powers);
        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

//public void breakFollowing() {
//    currentSplineToFollow.points.clear();
//}

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

//public void setSpline(Spline spline) {
//    currentSplineToFollow = spline;
//}

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