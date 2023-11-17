package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

public class PriorityServo extends PriorityDevice{
    public enum ServoType {
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12),
        AXON_MINI(0.173623, Math.toRadians(330) / 1.05); //todo

        public double positionPerRadian;
        public double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public final Servo[] servo;
    public ServoType type;
    public final double minPos, minAng, maxPos, maxAng, basePos;
    protected double currentAngle = 0, targetAngle = 0, power = 0;
    protected boolean reachedIntermediate = false;
    protected double currentIntermediateTargetAngle = 0;
    protected double[] multipliers;
    private long lastLoopTime = System.nanoTime();
    public String name;

    public PriorityServo(Servo servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale) {
        this(new Servo[] {servo}, name, type, loadMultiplier, min, max, basePos, reversed, basePriority, priorityScale);
    }

    public PriorityServo(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale, double[] multipliers) {
        this(servo,name,type,loadMultiplier,min,max,basePos,reversed,basePriority,priorityScale);
        this.multipliers = multipliers;
    }

    public PriorityServo(Servo[] servo, String name, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale) {
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

    public void setTargetPose(double targetPose, double power){
        setTargetAngle(convertPosToAngle(targetPose),power);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public void setTargetAngle(double targetAngle, double power){
        this.power = power;
        this.targetAngle = Math.max(Math.min(targetAngle,maxAng),minAng);
        TelemetryUtil.packet.put("target", targetAngle);
    }

    public void updateServoValues() {
        //updates the current angle the servo thinks it is at
        long currentTime = System.nanoTime();
        double loopTime = ((double) currentTime - lastLoopTime)/1.0E9;
        double error = currentIntermediateTargetAngle - currentAngle;
        double deltaAngle = loopTime * type.speed * power * Math.signum(error);
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

        return (reachedIntermediate ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.nanoTime() - lastUpdateTime)/1000.0 * priorityScale;
    }

    @Override
    public void update(){
        //Finds the amount of time since the intermediate target variable has been updated
        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = ((double) currentTime - lastUpdateTime)/1.0E9;

        double error = targetAngle - currentAngle;
        //find the amount of movement the servo theoretically should have done in the time it took to update the servo
        double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);

        if (Math.abs(deltaAngle) > Math.abs(error)){ //make sure that the servo doesn't ossilate over target
            deltaAngle = error;
        }

        currentIntermediateTargetAngle += deltaAngle; // adds the change in pose to the target for the servo
        if (power == 1){
            currentIntermediateTargetAngle = targetAngle; // makes it so that it goes to the end if the power is 1.0 ie no slow downs
        }

        for (int i = 0; i < servo.length; i++) {
            servo[i].setPosition(convertAngleToPos(currentIntermediateTargetAngle * multipliers[i])); //sets the servo to actual move to the target
        }
        lastUpdateTime = currentTime;
    }

    public double getTargetPosition () {
        return convertAngleToPos(currentIntermediateTargetAngle);
    }
}
