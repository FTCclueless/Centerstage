package org.firstinspires.ftc.teamcode.utils.priority;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

public class PriorityServo extends PriorityDevice{
    public enum ServoType {
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AXON_MINI(0.1784612002049795, 5.6403953024772129),
        AXON_MAX(0.1775562245447108, 6.5830247235911042),
        AXON_MICRO(0.1775562245447108, 6.5830247235911042),  // TODO need to tune
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12),
        HITEC(0.2966648, 5.6403953024772129);

        public double positionPerRadian;
        public double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public Servo[] servo;
    public ServoType type;
    public double minPos, minAng, maxPos, maxAng, basePos;
    protected double currentAngle = 0, targetAngle = 0, power = 0;
    protected boolean reachedIntermediate = false;
    protected double currentIntermediateTargetAngle = 0;
    protected double[] multipliers = null;
    protected boolean reversed = false;
    private long lastLoopTime = System.nanoTime();
    public String name;
    public double slowdownDist;
    public double slowdownPow;

    //basic 1 servo constructor
    public PriorityServo(Servo servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale) {
        this(new Servo[] {servo}, name, type, loadMultiplier, min, max, basePos, reversed, 0, 1, basePriority, priorityScale);
    }

    //load multiplier multi servo constructor
    public PriorityServo(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale, double[] multipliers) {
        this(servo,name,type,loadMultiplier,min,max,basePos,reversed, 0, 1, basePriority,priorityScale);
        this.multipliers = multipliers;
    }

    public PriorityServo(Servo servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers) {
        this(new Servo[] {servo}, name, type, loadMultiplier, min,  max,  basePos,  reversed, slowdownDist, slowdownPow,  basePriority,  priorityScale);
        this.multipliers = multipliers;
    }

    public PriorityServo(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale, double [] multipliers) {
        this(servo, name, type, loadMultiplier, min,  max,  basePos,  reversed, slowdownDist, slowdownPow,  basePriority,  priorityScale);
        this.multipliers = multipliers;
    }

    public PriorityServo(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale) {
        super(basePriority,priorityScale, name);
        this.servo = servo;
        this.type = type;
        this.type.speed *= loadMultiplier;
        this.name = name;
        this.reversed = reversed;
        if (reversed) {
            this.type.positionPerRadian *= -1;
        }

        this.basePos = basePos;

        minPos = Math.min(min,max);
        maxPos = Math.max(min,max);

        minAng = convertPosToAngle(minPos);
        maxAng = convertPosToAngle(maxPos);

        callLengthMillis = 1.0;

        multipliers = new double[servo.length];
        for (int i = 0; i < servo.length; i++) {
            multipliers[i] = 1;
            //servo[i].getController().pwmEnable(); turn on if u want -- Eric
        }

        this.slowdownDist = slowdownDist;
        this.slowdownPow = slowdownPow;
    }

    public double convertPosToAngle(double pos){
        pos -= basePos;
        pos /= type.positionPerRadian;
        return pos;
    }
    public double convertAngleToPos(double ang){
        ang *= type.positionPerRadian;
        ang += basePos;
        return ang;
    }

    public void setTargetPose(double targetPose, double power) {
        setTargetAngle(convertPosToAngle(targetPose),power);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public void setTargetAngle(double targetAngle, double power){
        this.power = power;
        this.targetAngle = Math.max(Math.min(targetAngle,maxAng),minAng);
    }

    public void updateServoValues() {
        //updates the current angle the servo thinks it is at
        long currentTime = System.nanoTime();
        double loopTime = ((double) currentTime - lastLoopTime)/1.0E9;
        double error = currentIntermediateTargetAngle - currentAngle;
        double deltaAngle = loopTime * type.speed * (Math.abs(error) <= slowdownDist ? slowdownPow : power) * Math.signum(error);
        //double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);
        reachedIntermediate = Math.abs(deltaAngle) > Math.abs(error);
        if (reachedIntermediate){
            deltaAngle = error;
        }
        currentAngle += deltaAngle;
        lastLoopTime = currentTime;
    }

    public void setCurrentAngle(double currentAngle) {
        this.currentAngle = currentAngle;
//        this.targetAngle = currentAngle;
//        this.currentIntermediateTargetAngle=currentAngle;
//
//        for (int i = 0; i < servo.length; i++) {
//            if (multipliers[i] == 1) {
//                servo[i].setPosition(convertAngleToPos(currentAngle)); //sets the servo to actual move to the target
//            } else {
//                servo[i].setPosition(maxPos - convertAngleToPos(currentAngle)); //this might be completely wrong --Kyle
//            }
//        }
    }

    public boolean inPosition(){
        return Math.abs(targetAngle-currentAngle) < Math.toRadians(0.01);
    }

    @Override
    public double getPriority(double timeRemaining) {
        if (isUpdated) {
            return 0;
        }

        updateServoValues();

        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }

        double priority = ((reachedIntermediate && currentIntermediateTargetAngle != targetAngle) ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;

        if (priority == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        return priority;
    }

    @Override
    public void update() {
        //Finds the amount of time since the intermediate target variable has been updated
        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = ((double) currentTime - lastUpdateTime)/1.0E9;

        double error = targetAngle - currentAngle;
//        Log.e(name, "error: " + error);
        //find the amount of movement the servo theoretically should have done in the time it took to update the servo
        double deltaAngle = timeSinceLastUpdate * type.speed * (Math.abs(error) <= slowdownDist ? slowdownPow : power) * Math.signum(error);
        //double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);

        if (Math.abs(deltaAngle) > Math.abs(error)){ //make sure that the servo doesn't ossilate over target
            deltaAngle = error;
        }

        currentIntermediateTargetAngle += deltaAngle; // adds the change in pose to the target for the servo
        if (power == 1) {
            currentIntermediateTargetAngle = targetAngle;
            if (slowdownDist != 0 && Math.abs(error) > slowdownDist + Math.toRadians(15)) {
                currentIntermediateTargetAngle = targetAngle-slowdownDist*Math.signum(error); // makes it so that it goes to the end if the power is 1.0 ie no slow downs
                //currentIntermediateTargetAngle = targetAngle;
            }
        }

//        if (Math.abs(error) < Math.toRadians(15)){
//            currentIntermediateTargetAngle = targetAngle;
//        }

        for (int i = 0; i < servo.length; i++) {
            if (multipliers[i] == 1) {
                servo[i].setPosition(convertAngleToPos(currentIntermediateTargetAngle)); //sets the servo to actual move to the target
            } else {
                servo[i].setPosition(maxPos - convertAngleToPos(currentIntermediateTargetAngle)); //this might be completely wrong --Kyle
            }
        }

        isUpdated = true;
        lastUpdateTime = currentTime;
    }

    public double getTargetPosition () {
        return convertAngleToPos(currentIntermediateTargetAngle);
    }
    public double getTargetAngle() {
        return currentIntermediateTargetAngle;
    }

//    public void setTargetAngleFORCED(double angle) {
//        setTargetAngle(angle, 1.0);
//        update();
//    }
}
