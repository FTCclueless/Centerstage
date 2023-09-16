package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.MIN_MOTOR_POWER_TO_OVERCOME_FRICTION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.qol.Subsystem;

import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain extends Subsystem {
    // Pure pursuit tuning values
    enum State {
        FOLLOW_SPLINE,
        GO_POINT,
        DONE
    }

    public static double maxRadius = 20;
    public static double headingCorrectionP = 0.8;
    public static double minRadius = 11;
    public static double maxCurve = 0.3;
    public static double turnMul = 1;
    public static double headingError = 5;
    public static double minSpeedFollowPath = 0.3;
    public static double slowdown = 0.25;

    double maxSpeed = 54;
    double maxTurn = maxSpeed / (TRACK_WIDTH);

    public PriorityMotor leftFront, leftRear, rightRear, rightFront;
    private List<PriorityMotor> motors;

    private HardwareQueue hardwareQueue;
    private Sensors sensors;

    public TwoWheelLocalizer localizer;

    private Spline currentPath = null;
    private Pose2d targetPoint = null;
    private int pathIndex = 0;

    public Drivetrain(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        super(State.FOLLOW_SPLINE);
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
            //motors.get(i).getMotorType().setAchieveableMaxRPMFraction(1.0);


            // Boopy coding brr
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

        localizer = new TwoWheelLocalizer(hardwareMap);

        addState(State.FOLLOW_SPLINE, () -> {
            Canvas canvas = TelemetryUtil.packet.fieldOverlay();
            Pose2d estimate = localizer.getPoseEstimate();

            //Find the closest point on the path to the robot
            double distanceToClosestPoint = currentPath.poses.get(pathIndex).getDistanceFromPoint(estimate);
            double distanceToNextPoint = currentPath.poses.get(Math.min(pathIndex + 1, currentPath.poses.size() - 1)).getDistanceFromPoint(estimate);
            while (distanceToClosestPoint > distanceToNextPoint && pathIndex < currentPath.poses.size() - 1) {
                pathIndex++;
                distanceToClosestPoint = distanceToNextPoint;
                distanceToNextPoint = currentPath.poses.get(Math.min(pathIndex + 1, currentPath.poses.size() - 1)).getDistanceFromPoint(estimate);
            }
            //Find the target for the robot to go to
            double pathCurvyness = 0;
            int targetIndex = pathIndex;
            double targetRadius = currentPath.poses.get(targetIndex).getDistanceFromPoint(estimate);
            double lastRadius = targetRadius;
            while ((pathCurvyness < maxCurve || targetRadius < minRadius) && targetRadius < maxRadius && targetIndex < currentPath.poses.size() - 1) {
                pathCurvyness += 1 / Math.max(currentPath.poses.get(targetIndex).radius, 8);
                targetIndex++;
                lastRadius = targetRadius;
                targetRadius = currentPath.poses.get(targetIndex).getDistanceFromPoint(estimate);
                if (targetRadius < lastRadius) { //If the radius becomes smaller it means you are missing part of the trajectory and therefore we do this so that it finds the next point again
                    targetIndex--;
                    targetRadius = lastRadius;
                    break;
                }
            }
            //this kinda jank but will leave for now

        /*if (currentPath.poses.get(pathIndex).reversed) {
            estimate.heading += Math.PI;
        }*/

            Pose2d lookAhead = currentPath.poses.get(targetIndex);

            TelemetryUtil.packet.put("pathIndex", pathIndex + "/" + currentPath.poses.size());

            // Plot the lookahead point
            canvas.setFill("#ff0000");
            canvas.fillCircle(lookAhead.x, lookAhead.y, 1.5);

            Pose2d error = new Pose2d(
                    lookAhead.x - estimate.x,
                    lookAhead.y - estimate.y,
                    AngleUtil.clipAngle(lookAhead.heading - estimate.heading + (currentPath.poses.get(pathIndex).reversed ? Math.PI : 0))
            );

            double relativeErrorY = error.y * Math.cos(estimate.heading) - error.x * Math.sin(estimate.heading);
            double relativeErrorX = error.x * Math.cos(estimate.heading) + error.y * Math.sin(estimate.heading); // why calculate it like this??????


            TelemetryUtil.packet.put("rel_error", relativeErrorX + " " + relativeErrorY);

            double radius = (error.x * error.x + error.y * error.y) / (2 * relativeErrorY);
            radius *= Math.signum(relativeErrorX);
            double theta = Math.atan2(relativeErrorY, relativeErrorX);

            // Plot the circle thing
            Vector2 perp = new Vector2(-Math.sin(estimate.heading), Math.cos(estimate.heading));
            if (Math.abs(radius) < 25) { // Don't put radius if it will explode ftc dashboard
                canvas.setStroke("#0000ff");
                perp.norm();
                perp.mul(radius);
                perp.mul(Math.signum(radius));
                perp.add(new Vector2(estimate.x, estimate.y));
                canvas.strokeLine(estimate.x, estimate.y, perp.x, perp.y);
                canvas.strokeCircle(perp.x, perp.y, Math.abs(radius));
            }

            TelemetryUtil.packet.put("Reversed", currentPath.poses.get(pathIndex).reversed);
            double speed = targetRadius > minRadius ?
                    (targetRadius - minRadius) / (maxRadius - minRadius) * (1.0 - minSpeedFollowPath) + minSpeedFollowPath :
                    (Math.abs(relativeErrorX) / minRadius) * (minSpeedFollowPath - slowdown) + slowdown; //Find the speed based on the radius -> determined by the curvyness of the path infront of robot
            double targetFwd = speed * (Math.abs(relativeErrorX) > 0.5 ? Math.signum(relativeErrorX) : 0);
            double targetTurn = speed * (targetRadius > minRadius ?
                    (TRACK_WIDTH / 2.0) / radius :
                    error.heading * headingCorrectionP);

            TelemetryUtil.packet.put("error heading", error.heading);


            if (pathIndex >= currentPath.poses.size() - 1 && Math.abs(error.heading) < Math.toRadians(headingError)) {
                setTargetPoint(currentPath.getLastPoint());
                currentPath = null;
                state = State.GO_POINT;
                return;
            }

            //apply the feedforward
            double fwd = targetFwd + (targetFwd - localizer.relCurrentVel.x / maxSpeed) * 0.35;
            double turn = targetTurn + (targetTurn - localizer.relCurrentVel.heading / maxTurn) * 0.2;
            turn *= turnMul;
            double[] motorPowers = {
                    fwd - turn,
                    fwd - turn,
                    fwd + turn,
                    fwd + turn
            };
            TelemetryUtil.packet.put("fwd", fwd);
            TelemetryUtil.packet.put("turn", turn);
            TelemetryUtil.packet.put("radius", radius);

            // Post 1 normalization
            double max = 1;
            for (double power : motorPowers) {
                max = Math.max(max, power);
            }

            for (int i = 0; i < motors.size(); i++) {
                motorPowers[i] /= max;
                motorPowers[i] *= 1.0 - MIN_MOTOR_POWER_TO_OVERCOME_FRICTION; // we do this so that we keep proportions when we add MIN_MOTOR_POWER_TO_OVERCOME_FRICTION in the next line below. If we had just added MIN_MOTOR_POWER_TO_OVERCOME_FRICTION without doing this 0.9 and 1.0 become the same motor power
                motorPowers[i] += MIN_MOTOR_POWER_TO_OVERCOME_FRICTION * Math.signum(motorPowers[i]);
                TelemetryUtil.packet.put("Max", max);
                TelemetryUtil.packet.put("Motor power", motorPowers[0] + " " + motorPowers[1] + " " + motorPowers[2] + " " + motorPowers[3]);

                //motors.get(i).setPower(motorPowers[i]);
                motors.get(i).setTargetPower(motorPowers[i]);
            }
        });

        addState(State.GO_POINT, () -> {
            goToPoint();
            if (Math.sqrt(Math.pow((localizer.getPoseEstimate().x - targetPoint.x),2) + Math.pow(localizer.getPoseEstimate().y,2)) < 1 && Math.abs(targetPoint.heading-localizer.heading) < 0.5 ) {
                state = State.DONE;
            }
        });

        addState(State.DONE, () -> {
            for (int i =0; i < motors.size(); i++) {
                motors.get(i).setTargetPower(0);
            }
        });
    }

    public void setCurrentPath(Spline path) {
        currentPath = path;
        pathIndex = 0;
    }

    public Spline getCurrentPath() {
        return currentPath;
    }

    double maxHeadingError = Math.toRadians(95);

    public void update() {
        super.update();
        updateLocalizer();

        if (currentPath != null) {
            state = State.FOLLOW_SPLINE;
        }

        Pose2d estimate = localizer.getPoseEstimate();
        ROBOT_POSITION = new Pose2d(estimate.x, estimate.y, estimate.heading);
        ROBOT_VELOCITY = localizer.getPoseVelocity();
    }

    public void setTargetPoint(Pose2d point) {
        targetPoint = point;
    }

    public static double kx = 0.2; //todo tune these
    public static double ky = 0.2;
    public static double kang = 0.2;
    public void goToPoint() {

        double fwd = kx*(targetPoint.x - localizer.x);
        double strafe = ky*(targetPoint.y-localizer.y);
        double turn = kang*(targetPoint.heading-localizer.heading);


    }

    public void updateLocalizer() {
        localizer.updateEncoders(sensors.getOdometry());
        localizer.update();
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

    public void moveVector(Vector2 direction, Vector2 turn) {
        return; //todo
    }

    public void drive(Gamepad gamepad) {
        //double forward = 0.45 * Math.tan(((gamepad.left_stick_y * -1) / 0.85));
        //double strafe = 0.45* Math.tan((gamepad.left_stick_x * -1)/0.85);
        double forward = gamepad.left_stick_y * -1;
        double strafe = gamepad.right_stick_y;
        TelemetryUtil.packet.put("forward", forward);
        double turn = 0.8 * gamepad.right_stick_x;
        TelemetryUtil.packet.put("turn", turn);

        double p1 = forward + turn + strafe;
        double p2 = forward + turn - strafe;
        double p3 = forward - turn - strafe;
        double p4 = forward - turn + strafe;
        setMotorPowers(p1, p2, p3, p4);
    }

    /*boolean breakFollowing = false;
    Pose2d targetPose = new Pose2d(0, 0, 0);
    double xThreshold = 0.5;
    double yThreshold = 0.5;
    double headingThreshold = Math.toRadians(5.0);

    public void setBreakFollowingThresholds(Pose2d thresholds, Pose2d targetPose) {
        this.targetPose = targetPose;
        breakFollowing = true;
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        headingThreshold = thresholds.getHeading();
    }*/

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

    public boolean isBusy() {
        return currentPath == null;
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