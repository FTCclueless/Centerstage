package org.firstinspires.ftc.teamcode.subsystems.hang;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Hang {
    private final PriorityCRServo hang;
    private final PriorityServo leftArm;
    private final PriorityServo rightArm;

    private int state = 0;

    double leftDownAngle = 1.09267;
    double leftHalfwayAngle = 1.681;
    double leftUpAngle = 2.65043;

    double rightDownAngle = 0.4258;
    double rightHalfwayAngle = 1.1487;
    double rightUpAngle = 1.9444;

    public Hang(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        CRServo leftHang = hardwareMap.get(CRServo.class, "leftHang");
        CRServo rightHang = hardwareMap.get(CRServo.class, "rightHang");
        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);

        hang = new PriorityCRServo(new CRServo[]{leftHang, rightHang}, "hang", 1, 2);
        leftArm = new PriorityServo(hardwareMap.get(Servo.class, "leftArm"),
                "leftArm",
                PriorityServo.ServoType.AXON_MINI,
                1.0,
                0.0,
                1.0,
                0.5,
                false,
                1.0,
                1.0
        );
        rightArm = new PriorityServo(hardwareMap.get(Servo.class, "rightArm"),
                "rightArm",
                PriorityServo.ServoType.AXON_MINI,
                1.0,
                0.0,
                1.0,
                0.5,
                false,
                1.0,
                1.0
        );

        hardwareQueue.addDevice(hang);
        hardwareQueue.addDevice(leftArm);
        hardwareQueue.addDevice(rightArm);
    }

    boolean alreadyReversed = false;
    long startReverse = System.currentTimeMillis();
    public void nextHangState() {
        if (state == 0 || state == 2) { // we are in arms down/halfway rest points
            startReverse = System.currentTimeMillis();
        }
        if (state != 1 && state != 3) { // currently reversing states
            state++;
        }

        if (state > 4) {
            state = 0;
        }
    }

    public void update() {
        switch (state) {
            case 0: // arms down rest point
                armsDown();
                break;
            case 1: // arms halfway
                state = 2;
                armsHalfway(1.0);
                break;
            case 2: // arms halfway rest point
                armsHalfway(1.0);
                break;
            case 3: // arms up
                state = 4;
                armsUp(0.75);
                reverse();
                break;
            case 4: // arms up now and ready to hang
                alreadyReversed = true;
                break;
        }
    }
    public void on() {
        hang.setTargetPower(1.0);
    }

    public void reverse() {
        hang.setTargetPower(-1.0);
    }

    public void off() {
        hang.setTargetPower(0.0);
    }

    public boolean doingHang() {
        return state == 4;
    }

    private void armsDown() {
        leftArm.setTargetAngle(leftDownAngle,1.0);
        rightArm.setTargetAngle(rightDownAngle, 1.0);
    }

    private void armsHalfway(double power) {
        leftArm.setTargetAngle(leftHalfwayAngle,power);
        rightArm.setTargetAngle(rightHalfwayAngle, power);
    }

    private void armsUp(double power) {
        leftArm.setTargetAngle(leftUpAngle,power);
        rightArm.setTargetAngle(rightUpAngle, power);
    }
}
