package org.firstinspires.ftc.teamcode.subsystems.hang;

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

    private State state = State.OFF;
    double leftDownAngle = 0.0;
    double leftHalfwayAngle = 0.0;
    double leftUpAngle = 0.0;

    double rightDownAngle = 0.0;
    double rightHalfwayAngle = 0.0;
    double rightUpAngle = 0.0;

    private boolean doingHang = false;

    enum State {
        ON,
        REVERSE,
        OFF
    }

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

    public void update() {
        switch (state) {
            case ON:
                hang.setTargetPower(1.0);
                break;
            case REVERSE:
                hang.setTargetPower(-1.0);
                break;
            case OFF:
                hang.setTargetPower(0.0);
                break;
        }
    }

    public void on() {
        state = State.ON;
    }

    public void reverse() {
        state = State.REVERSE;
    }

    public void off() {
        state = State.OFF;
    }

    public boolean doingHang() {
        return doingHang;
    }

    private void armsDown() {
        doingHang = false;
        leftArm.setTargetAngle(leftDownAngle,1.0);
        rightArm.setTargetAngle(rightDownAngle, 1.0);
    }

    private void armsHalfway() {
        doingHang = true;
        leftArm.setTargetAngle(leftHalfwayAngle,1.0);
        rightArm.setTargetAngle(rightHalfwayAngle, 1.0);
    }

    private void armsUp() {
        doingHang = true;
        leftArm.setTargetAngle(leftUpAngle,1.0);
        rightArm.setTargetAngle(rightUpAngle, 1.0);
    }

    int armIndex = 0;
    public void nextArmState() {
        armIndex++;
        if (armIndex > 2) {
            armIndex = 0;
        }

        switch (armIndex) {
            case 0:
                armsDown();
                break;
            case 1:
                armsHalfway();
                break;
            case 2:
                armsUp();
                break;
        }
    }
}
