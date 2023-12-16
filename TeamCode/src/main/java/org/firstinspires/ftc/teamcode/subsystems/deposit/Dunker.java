package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
// If you are reading this it is YOUR job to recode this hunk of junk!
public class Dunker {
    public PriorityServo dunker;

    public static double intakeAng = 2.31256;
    public static double lockAng = 2.128;
    public static double depoAng = 3.51;
    public static double dunkTime = 500;
    public static double timer = 0;
    private boolean anotherPixel = false;

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
        dunker.setTargetAngle(intakeAng, 1);
    }

    public void lock() {
        dunker.setTargetAngle(lockAng, 1);
    }

    public void dunk1() {
        timer = System.currentTimeMillis();
        if (anotherPixel) {
            dunker.setTargetAngle(intakeAng, 1);
            anotherPixel = false;
        } else {
            dunker.setTargetAngle(depoAng, 1);
        }
    }

    public void dunk2() {
        anotherPixel = false;
        this.dunk1();
        anotherPixel = true;
    }

    // Send help this function is actually so bad
    public boolean busy() {
        return timer != 0;
    }

    public void update() {
        if (System.currentTimeMillis() - timer >= dunkTime && timer != 0) {
            if (anotherPixel) {
                this.dunk1();
            } else {
                timer = 0;
            }
        }
    }
}
