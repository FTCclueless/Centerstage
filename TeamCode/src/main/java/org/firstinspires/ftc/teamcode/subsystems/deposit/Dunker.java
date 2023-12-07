package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Dunker {
    public PriorityServo dunker;
    public static double dunkAng = 3.034; //TODO
    public static double lockAng = 2.52069;
    public static double openAng = 2.6502;

    public enum DunkState {
        WAIT1,
        WAIT2,
        CHILL,
    }
    public DunkState dunkState = DunkState.CHILL; //todo change to close later

    private double startTime = 0;
    private boolean oneDunk = true;

    public static double oneTime = 1;
    public static double twoTime = 2;

    public Dunker(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        dunker = new PriorityServo(hardwareMap.get(Servo.class, "dunker"), "dunker",
                PriorityServo.ServoType.SPEED,
                1,
                0,
                1,
                0,
                false,
                1, 2);
        hardwareQueue.addDevice(dunker);
    }

    public void intake() {
        dunker.setTargetAngle(openAng, 1);
    }

    public void close() {
        dunker.setTargetAngle(lockAng, 1);
    }

    public void dunk1() {
        startTime = System.nanoTime()/(1.0e9);
        dunker.setTargetAngle(dunkAng, 1);
        dunkState = DunkState.WAIT1;
        oneDunk = true;
    }
    public void dunk2() {
        startTime = System.nanoTime()/(1.0e9);
        dunker.setTargetAngle(dunkAng, 1);
        dunkState = DunkState.WAIT2;
        oneDunk= false;
    }

    boolean hasTime = false;
    public void update() {
        switch (dunkState) {
            case CHILL:
                break;
            case WAIT2:
                if (System.nanoTime()/1.0e9 - startTime >=  oneTime ) {
                    intake();
                }
            if (System.nanoTime()/1.0e9 - startTime >= twoTime) {
                dunk1();
            }
                break;
            case WAIT1:
                if (System.nanoTime()/1.0e9 - startTime >=  oneTime ) {
                    intake();
                    dunkState = DunkState.CHILL;
                }
                break;

        }
    }
}
