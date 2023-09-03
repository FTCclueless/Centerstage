package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

public class MotorPriority {
    double basePriority;
    double priorityScale;
    double lastPower = 0;
    public double power = 0;
    long lastUpdateTime;
    public DcMotorEx[] motor; // if the subsystem has multiple motors (i.e. slides)

    public MotorPriority(DcMotorEx a, double basePriority, double priorityScale){
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
        motor = new DcMotorEx[]{a};
    }
    public MotorPriority(DcMotorEx[] a, double basePriority, double priorityScale){
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
        motor = a;
    }

    public void setTargetPower(double targetPower){
        power = targetPower;
    }

    public double getPriority(double timeRemaining) { // timeRemaining is in secs
        if (power-lastPower == 0) {
            lastUpdateTime = System.currentTimeMillis();
            return 0;
        }
        // checks if the time remaining to be within targetLoopLength is less than 0.8 (for all motors except slides, since it is two motors).
        // if it is less than 0.8 then don't update, else update
        // Motors take 1.6ms to update so actual loopLength might be targetLoopLength + 0.8

        if (timeRemaining * 1000.0 <= 1.6 * (motor.length - 1) + 0.8) {// potentially is 1.6 ms but we use this formula to allow it to slightly overshoot/undershoot
            return 0;
        }

        return basePriority + Math.abs(power-lastPower) * (System.currentTimeMillis() - lastUpdateTime) * priorityScale;
    }

    public void update(){
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power);
        }
        lastUpdateTime = System.currentTimeMillis();
        lastPower = power;
    }

    public static void updateMotors(ArrayList<MotorPriority> motorPriorities) {
        double bestMotorUpdate = 1;
        double targetLoopLength = 0.015; // sets the target loop time in seconds
        double loopTime = GET_LOOP_TIME(); // finds loopTime in seconds

        while (bestMotorUpdate > 0 && loopTime <= targetLoopLength) { // updates the motors while still time remaining in the loop
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority(targetLoopLength - loopTime);

            // finds motor that needs updating the most
            for (int i = 1; i < motorPriorities.size(); i++) { //finding the motor that is most in need of being updated;
                double currentMotor = motorPriorities.get(i).getPriority(targetLoopLength - loopTime);
                if (currentMotor > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = currentMotor;
                }
            }
            if (bestMotorUpdate != 0) { // priority # of motor needing update the most
                motorPriorities.get(bestIndex).update(); // Resetting the motor priority so that it knows that it updated the motor and setting the motor of the one that most needs it
            }
            loopTime = GET_LOOP_TIME();
        }
    }
}
