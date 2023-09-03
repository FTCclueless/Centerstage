package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    public enum ServoType {
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12);

        public double positionPerRadian;
        public double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public final Servo servo;
    public ServoType type;
    private final double minPos, minAng, maxPos, maxAng, basePos;
    private double currentAngle = 0, targetAngle = 0, power = 0;
    private double basePriority, priorityScale;
    private boolean reachedIntermediate = false;
    private double lastUpdatedTargetAngle = 0, currentIntermediateTargetAngle = 0;
    private long lastUpdateTime = System.nanoTime();
    private long lastLoopTime = System.nanoTime();

    public MyServo(Servo servo, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed) {
        this(servo,type,loadMultiplier,min,max,basePos,reversed, 2, 3);
    }

    public MyServo(Servo servo, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed, double basePriority, double priorityScale) {
        this.servo = servo;
        this.type = type;
        this.type.speed *= loadMultiplier;
        if (reversed) {
            this.type.positionPerRadian *= -1;
        }

        this.basePos = basePos;

        minPos = Math.min(min,max);
        maxPos = Math.max(min,max);

        minAng = convertPosToAngle(minPos);
        maxAng = convertPosToAngle(maxPos);

        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
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

    public void setTargetAngle(double targetAngle, double power){
        this.power = power;
        this.targetAngle = Math.max(Math.min(targetAngle,maxAng),minAng);
    }

    public void update(){
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

    public double getPriority(double timeRemaining) { // timeRemaining is in secs
        if (targetAngle-currentIntermediateTargetAngle == 0) {
            lastUpdateTime = System.currentTimeMillis();
            return 0;
        }
        // checks if the time remaining to be within targetLoopLength is less than 0.8 (for all motors except slides, since it is two motors).
        // if it is less than 0.8 then don't update, else update
        // Motors take 1.6ms to update so actual loopLength might be targetLoopLength + 0.8

        if (timeRemaining * 1000.0 <= 0.5) {// potentially is 1 ms but we use this formula to allow it to slightly overshoot/undershoot
            return 0;
        }

        return (reachedIntermediate ? basePriority : 0) + Math.abs(targetAngle-currentIntermediateTargetAngle) * (System.currentTimeMillis() - lastUpdateTime) * priorityScale;
    }

    public void updateServoTarget(){
        //Finds the amount of time since the intermediate target variable has been updated
        long currentTime = System.nanoTime();
        double timeSinceLastUpdate = ((double) currentTime - lastUpdateTime)/1.0E9;
        //update the lastUpdatedTargetAngle so it is inline with its current target
        lastUpdatedTargetAngle = targetAngle;
        double error = lastUpdatedTargetAngle - currentAngle;
        //find the amount of movement the servo theoretically should have done in the time it took to update the servo
        double deltaAngle = timeSinceLastUpdate * type.speed * power * Math.signum(error);
        if (Math.abs(deltaAngle) > Math.abs(error)){ //make sure that the servo doesn't ossilate over target
            deltaAngle = error;
        }

        currentIntermediateTargetAngle += deltaAngle; // adds the change in pose to the target for the servo
        if (power == 1){
            currentIntermediateTargetAngle = targetAngle; // makes it so that it goes to the end if the power is 1.0 ie no slow downs
        }

        servo.setPosition(convertAngleToPos(currentIntermediateTargetAngle)); //sets the servo to actual move to the target
        lastUpdateTime = currentTime;
    }
}