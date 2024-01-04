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

    double leftDownAngle = 0.73965;
    double leftHalfwayAngle = 1.4849;
    double leftUpAngle = 2.31983;

    double rightDownAngle = -1.159915;
    double rightHalfwayAngle = -0.397845;
    double rightUpAngle = 0.42586;

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

    boolean isReversing = false;
    boolean alreadyReversed = false;
    long startReverse = System.currentTimeMillis();
    public void nextHangState() {
        if (state == 0) { // we are in arms down
            state++;
            isReversing = true;
            startReverse = System.currentTimeMillis();
        }
        if (!isReversing) {
            state++;
        }

        if (state > 4) {
            state = 0;
        }
    }

    public void update() {
        switch (state) {
            case 0: // arms down
                armsDown();
                break;
            case 1: // reverse hang
                if (alreadyReversed) {
                    state = 3;
                }
                if (System.currentTimeMillis() - startReverse > 2500) {
                    isReversing = false;
                    state = 2;
                    off();
                } else {
                    reverse();
                }
                break;
            case 2: // reversed
                off();
                isReversing = false;
                alreadyReversed = true;
                break;
            case 3: // arms halfway
                isReversing = false;
                alreadyReversed = true;
                off();
                armsHalfway();
                break;
            case 4: // arms up
                armsUp();
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

    private void armsHalfway() {
        leftArm.setTargetAngle(leftHalfwayAngle,1.0);
        rightArm.setTargetAngle(rightHalfwayAngle, 1.0);
    }

    private void armsUp() {
        leftArm.setTargetAngle(leftUpAngle,1.0);
        rightArm.setTargetAngle(rightUpAngle, 1.0);
    }
}
