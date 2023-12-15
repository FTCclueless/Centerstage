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
    public static double openAng = 2.25;
    public static double depoAng = 3.57;
    public static double dunkTime = 1000;
    public static double startTime = 0;

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

    public void dunk2() {
        startTime = System.currentTimeMillis();
        dunker.setTargetAngle(depoAng, 1);
    }

    // Send help this function is actually so bad
    public boolean busy() {
        return dunker.getCurrentAngle() != dunker.getTargetAngle();
    }

    public void update() {
        if (System.currentTimeMillis() - startTime >= dunkTime && startTime != 0) {
            dunker.setTargetAngle(openAng, 1);
            startTime = 0;
        }
    }
}
