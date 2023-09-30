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
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain {
    // Pure pursuit tuning values

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

    public ThreeWheelLocalizer localizer;

    private Spline currentPath = null;
    private int pathIndex = 0;

    public Drivetrain(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
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

        localizer = new ThreeWheelLocalizer(hardwareMap, false);
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
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        updateLocalizer();

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        Pose2d estimate = localizer.getPoseEstimate();
        ROBOT_POSITION = new Pose2d(estimate.x, estimate.y,estimate.heading);
        ROBOT_VELOCITY = localizer.getPoseVelocity();

        if (currentPath != null) {
            //Find the closest point on the path to the robot
            double distanceToClosestPoint = currentPath.poses.get(pathIndex).getDistanceFromPoint(estimate);
            double distanceToNextPoint = currentPath.poses.get(Math.min(pathIndex + 1,currentPath.poses.size() - 1)).getDistanceFromPoint(estimate);
            while (distanceToClosestPoint > distanceToNextPoint && pathIndex < currentPath.poses.size() - 1) {
                pathIndex++;
                distanceToClosestPoint = distanceToNextPoint;
                distanceToNextPoint = currentPath.poses.get(Math.min(pathIndex + 1,currentPath.poses.size() - 1)).getDistanceFromPoint(estimate);
            }
            //Find the target for the robot to go to
            double pathCurvyness = 0;
            int targetIndex = pathIndex;
            double targetRadius = currentPath.poses.get(targetIndex).getDistanceFromPoint(estimate);
            double lastRadius = targetRadius;
            while ((pathCurvyness < maxCurve || targetRadius < minRadius) && targetRadius < maxRadius && targetIndex < currentPath.poses.size() - 1){
                pathCurvyness += 1/Math.max(currentPath.poses.get(targetIndex).radius,8);
                targetIndex ++;
                lastRadius = targetRadius;
                targetRadius = currentPath.poses.get(targetIndex).getDistanceFromPoint(estimate);
                if (targetRadius < lastRadius){ //If the radius becomes smaller it means you are missing part of the trajectory and therefore we do this so that it finds the next point again
                    targetIndex --;
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
                    (targetRadius-minRadius)/(maxRadius-minRadius)*(1.0 - minSpeedFollowPath) + minSpeedFollowPath :
                    (Math.abs(relativeErrorX)/minRadius)*(minSpeedFollowPath - slowdown) + slowdown; //Find the speed based on the radius -> determined by the curvyness of the path infront of robot
            double targetFwd = speed * (Math.abs(relativeErrorX) > 0.5 ? Math.signum(relativeErrorX) : 0);
            double targetTurn = speed * (targetRadius > minRadius ?
                    (TRACK_WIDTH / 2.0) / radius :
                    error.heading * headingCorrectionP);

            TelemetryUtil.packet.put("error heading", error.heading);


            if (pathIndex >= currentPath.poses.size() - 1 && Math.abs(error.heading) < Math.toRadians(headingError)) {
                currentPath = null;
                for (PriorityMotor motor : motors) {
                    motor.setTargetPower(0);
                }
                return;
            }

            //apply the feedforward
            double fwd = targetFwd + (targetFwd - localizer.relCurrentVel.x/maxSpeed) * 0.35;
            double turn = targetTurn + (targetTurn - localizer.relCurrentVel.heading/maxTurn) * 0.2 ;
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
        }

        /*// pure pursuit follower
        if (error != null) {
            double errorDistance = Math.sqrt(Math.pow(error.x,2) + Math.pow(error.y,2)); // distance equation
            boolean mustGoToPoint = (currentSplineToFollow.points.get(0).mustGoToPoint || currentSplineToFollow.points.size() == 1) && errorDistance < 10.0;
            double headingError = mustGoToPoint ? error.heading : Math.atan2(error.y,error.x) + currentSplineToFollow.points.get(0).headingOffset; // if we want to go to point then we go to the heading otherwise we point to point
            headingError = AngleUtil.clipAngle(headingError);

            double maxRadius = Pose.maxDistanceFromPoint;
            double minRadius = Pose.minDistanceFromPoint;
            double smallestRadiusOfNextPoints = currentSplineToFollow.points.get(0).radius;
            TelemetryUtil.packet.put("radius", smallestRadiusOfNextPoints);
            //for (int i = 1; i < Math.min(currentSplineToFollow.points.size()-1,1); i++)  { // finding smallest radius for next 5 points
            //    smallestRadiusOfNextPoints = Math.min(currentSplineToFollow.points.get(i).radius,smallestRadiusOfNextPoints);
            //}

            double speedFromRadiusPercentage = (smallestRadiusOfNextPoints-minRadius)/(maxRadius-minRadius); // Maximum forward speed based on the upcoming radius
            double speedFromHeadingErrorPercentage = Math.max((maxHeadingError - Math.abs(headingError))/maxHeadingError,0); // Maximum forward speed based on the current heading error
            double speedFromEndPercentage = mustGoToPoint ? Math.abs(error.x) / speedFromEndDiv : 1; // slows down the robot when it reaches an end

            double fwdSpeedPercentage = Math.min(speedFromRadiusPercentage,speedFromHeadingErrorPercentage);
            fwdSpeedPercentage = speedFromEndPercentage * Math.max(Math.min(fwdSpeedPercentage,maxSpeed),minSpeed); // we want the speed to slow down as we approach the point & minimum max speed
            double currentFwdPercentage = Math.min(Math.abs(localizer.relCurrentVel.x/MAX_DRIVETRAIN_SPEED),1.0);
            double currentTurnPercentage = Math.min(Math.abs(localizer.relCurrentVel.heading/TRACK_WIDTH), 1.0);

            double breakingFactor = 0.45; // scale factor for how much you wanna weigh current forward percentage into braking
            double differenceBetweenSetAndActual = fwdSpeedPercentage - currentFwdPercentage;

            double fwd = Math.signum(error.x) * (fwdSpeedPercentage + Math.max(differenceBetweenSetAndActual * breakingFactor, facDICKS)); // applies breaking power to slow it down, most breaking power applied is -0.3
            double turn = TRACK_WIDTH/2*headingError; // s=r*theta
            turn *= turnMultiplier;
            if (Math.abs(headingError) > Math.toRadians(slowBelowDeg) && Math.abs(headingError) < Math.toRadians(slowAboveDeg) && currentTurnPercentage > slowPercentageThresh) {
                turn = -turnSlownessAfterTurn * Math.signum(turn);
            }

            double[] motorPowers = {
                    fwd - turn,
                    fwd - turn,
                    fwd + turn,
                    fwd + turn
            };
            double max = 1.0;
            for (double motorPower : motorPowers) { // finds max power if greater than 1.0
                max = Math.max(max, Math.abs(motorPower));
            }
            for (int i = 0; i < motorPowers.length; i ++) {
                motorPowers[i] /= max; // keeps proportions in tack by getting a percentage
                motorPowers[i] *= 1.0 - MIN_MOTOR_POWER_TO_OVERCOME_FRICTION; // we do this so that we keep proportions when we add MIN_MOTOR_POWER_TO_OVERCOME_FRICTION in the next line below. If we had just added MIN_MOTOR_POWER_TO_OVERCOME_FRICTION without doing this 0.9 and 1.0 become the same motor power
                motorPowers[i] += MIN_MOTOR_POWER_TO_OVERCOME_FRICTION * Math.signum(motorPowers[i]);
                motorPriorities.get(i).setTargetPower(motorPowers[i]);
            }
        }*/

        /*if((breakFollowing)
                && (Math.abs(estimate.getX() - targetPose.getX()) < xThreshold)
                && (Math.abs(estimate.getY() - targetPose.getY()) < yThreshold)
                && (Math.abs(estimate.getHeading() - targetPose.getHeading()) < headingThreshold)) {
            breakFollowing();
            setMotorPowers(0,0,0,0);
        }*/
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

    public void drive(Gamepad gamepad) {
        double forward = 0.45 * Math.tan(((gamepad.left_stick_y * -1) / 0.85));
        TelemetryUtil.packet.put("forward", forward);
        double turn = 0.8 * gamepad.right_stick_x;
        TelemetryUtil.packet.put("turn", turn);

        double p1 = forward + turn;
        double p2 = forward + turn;
        double p3 = forward - turn;
        double p4 = forward - turn;
        setMotorPowers(p1, p2, p3, p4);
    }

    boolean breakFollowing = false;
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