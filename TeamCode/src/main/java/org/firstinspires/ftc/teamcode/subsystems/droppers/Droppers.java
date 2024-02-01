package org.firstinspires.ftc.teamcode.subsystems.droppers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Droppers {
    public PriorityServo leftDropper, rightDropper;

    public static double leftDownAngle = -1.37780139;
    public static double leftReleaseAngle = -3.982230;

    public static double rightDownAngle = 1.0591530567;
    public static double rightReleaseAngle = 2.1321814;

    public enum STATE {
        IDLE,
        BUSY
    }
    public STATE state = STATE.IDLE;

    public Droppers(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        leftDropper = new PriorityServo(hardwareMap.get(Servo.class, "leftDropper"), "leftDropper",
                PriorityServo.ServoType.SUPER_SPEED,
                1,
                0,
                1,
                0.861,
                false,
                1, 2);
        rightDropper = new PriorityServo(hardwareMap.get(Servo.class, "rightDropper"), "rightDropper",
                PriorityServo.ServoType.SUPER_SPEED,
                1,
                0,
                1,
                0.539000,
                false,
                1, 2);
        hardwareQueue.addDevice(leftDropper);
        hardwareQueue.addDevice(rightDropper);
    }

    public void leftDown() {
        leftDropper.setTargetAngle(leftDownAngle, 1);
    }

    public void leftRelease() {
        leftDropper.setTargetAngle(leftReleaseAngle, 0.05);
    }

    public void rightDown() {
        rightDropper.setTargetAngle(rightDownAngle, 1);
    }

    public void rightRelease() {
        rightDropper.setTargetAngle(rightReleaseAngle, 0.05);
    }

    public void retractBoth() {
        leftRelease();
        rightRelease();
    }

    public void update() {
        switch(state) {
            case IDLE:
                if (leftDropper.getCurrentAngle() != leftDropper.getTargetAngle() || rightDropper.getCurrentAngle() != rightDropper.getTargetAngle()) {
                    state = STATE.BUSY;
                }
                break;
            case BUSY:
                if (leftDropper.getCurrentAngle() == leftDropper.getTargetAngle() || rightDropper.getCurrentAngle() == rightDropper.getTargetAngle()) {
                    state = STATE.IDLE;
                }
                break;
        }
    }
}
