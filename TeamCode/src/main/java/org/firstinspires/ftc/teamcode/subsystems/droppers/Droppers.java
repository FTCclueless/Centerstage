package org.firstinspires.ftc.teamcode.subsystems.droppers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Droppers {
    public PriorityServo leftDropper, rightDropper;

    public static double leftDownAngle = 3.783335022;
    public static double leftReleaseAngle = 0.286757596;

    public static double rightDownAngle = -4.14873489;
    public static double rightReleaseAngle = -0.5550147022;

    public Droppers(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        leftDropper = new PriorityServo(hardwareMap.get(Servo.class, "leftDropper"), "leftDropper",
                PriorityServo.ServoType.SUPER_SPEED,
                1,
                0,
                1,
                0.0,
                false,
                1, 2);
        rightDropper = new PriorityServo(hardwareMap.get(Servo.class, "rightDropper"), "rightDropper",
                PriorityServo.ServoType.SUPER_SPEED,
                1,
                0,
                1,
                1.0,
                false,
                1, 2);
        hardwareQueue.addDevice(leftDropper);
        hardwareQueue.addDevice(rightDropper);
    }

    public void leftDown() {
        leftDropper.setTargetAngle(leftDownAngle, 1);
    }

    public void leftRelease() {
        leftDropper.setTargetAngle(leftReleaseAngle, 1);
    }

    public void rightDown() {
        rightDropper.setTargetAngle(rightDownAngle, 1);
    }

    public void rightRelease() {
        rightDropper.setTargetAngle(rightReleaseAngle, 1);
    }

    public void retractBoth() {
        leftRelease();
        rightRelease();
    }

    public void update() {}
}
