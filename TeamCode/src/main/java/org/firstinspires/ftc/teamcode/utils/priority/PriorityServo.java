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
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12),
        AXON_MINI(0.162338041953733, Math.toRadians(330) / 3), //todo 0.173623,
        AXON_MINI_SCUFF(0.09135495634, Math.toRadians(330)/1.05),
        AXON_MINI_SCUFF_TURRET(0.203081707385258, Math.toRadians(330) / 3.75); //todo all speeds somehow

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

    public PriorityServo(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double slowdownDist, double slowdownPow, double basePriority, double priorityScale) {
        super(basePriority,priorityScale, name);
        this.servo = servo;
        this.type = type;
        this.type.speed *= loadMultiplier;
        this.name = name;
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
//        Log.e(name, "targetAngle " + targetAngle);
        TelemetryUtil.packet.put(name + " target angle", targetAngle);
        TelemetryUtil.packet.put(name + " current angle", currentAngle);
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

    @Override
    public double getPriority(double timeRemaining) {
        updateServoValues();

        if (targetAngle-currentAngle == 0) {
            lastUpdateTime = System.nanoTime();
            return 0;
        }

        if (timeRemaining * 1000.0 <= callLengthMillis/2.0) {
            return 0;
        }

        TelemetryUtil.packet.put("reachedIntermediate", reachedIntermediate);
        return (reachedIntermediate ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000000.0 * priorityScale;
    }

    @Override
    public void update(){
        System.out.println(name + " has been updated");

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
        if (power == 1 && Math.abs(error) > slowdownDist){
            currentIntermediateTargetAngle = targetAngle-slowdownDist*Math.signum(targetAngle); // makes it so that it goes to the end if the power is 1.0 ie no slow downs
            //currentIntermediateTargetAngle = targetAngle;
        }


        double sum = 0;
        for (int i = 0; i < servo.length; i++) {
            if (multipliers[i] == 1) {
                servo[i].setPosition(convertAngleToPos(currentIntermediateTargetAngle)); //sets the servo to actual move to the target
                //Log.e(name,convertAngleToPos(currentIntermediateTargetAngle) +" angleToPos 1");
                sum += convertAngleToPos(currentIntermediateTargetAngle);
            } else {
                servo[i].setPosition(maxPos - convertAngleToPos(currentIntermediateTargetAngle)); //this might be completely wrong --Kyle
                //Log.e(name,(maxPos - convertAngleToPos(currentIntermediateTargetAngle)) + " angleToPos -1");
                sum += maxPos - convertAngleToPos(currentIntermediateTargetAngle);
            }
        }

        //Log.e("sum, ", sum +"");
        lastUpdateTime = currentTime;
    }

    public double getTargetPosition () {
        return convertAngleToPos(currentIntermediateTargetAngle);
    }
}
