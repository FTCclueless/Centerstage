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
    public static double dunkPos = 0.146; //TODO
    public static double holdPos = 0.417;
    public enum DunkState {
        STARTDUNK,
        WAIT,
        CLOSE,
        CHILL
    }
    public DunkState dunkState = DunkState.CLOSE;

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

    public void dunk1() {
        dunkState = DunkState.STARTDUNK;
        oneDunk = true;
    }
    public void dunk2() {
        dunkState = DunkState.STARTDUNK;
        oneDunk= false;
    }

    boolean hasTime = false;
    public void update() {
        switch (dunkState) {
            case CLOSE:
                dunker.setTargetPose(holdPos, 0.7);
                dunkState = DunkState.CHILL;
                break;
            case CHILL:
                break;
            case STARTDUNK:
                dunker.setTargetPose(dunkPos, 0.01);
                dunkState = DunkState.WAIT;
                break;
            case WAIT:
                if (!hasTime) {
                    startTime = System.nanoTime()/(1.0e9);
                    hasTime = true;
                }
                else if (System.nanoTime()/1.0e9 - startTime >= (oneDunk ? oneTime : twoTime)) {
                    dunkState = DunkState.CLOSE;
                    hasTime = false;
                }
                break;
        }
    }
}
