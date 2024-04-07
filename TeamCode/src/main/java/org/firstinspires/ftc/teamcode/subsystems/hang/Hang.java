package org.firstinspires.ftc.teamcode.subsystems.hang;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

public class Hang {
    private final PriorityCRServo leftServo;
    private final PriorityCRServo rightServo;

    public Hang(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        CRServo leftHang = hardwareMap.get(CRServo.class, "leftHang");
        CRServo rightHang = hardwareMap.get(CRServo.class, "rightHang");
        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = new PriorityCRServo(new CRServo[]{leftHang}, "leftHang", 1, 2);
        rightServo = new PriorityCRServo(new CRServo[]{rightHang}, "rightHang", 1, 2);

        hardwareQueue.addDevice(leftServo);
        hardwareQueue.addDevice(rightServo);
    }

    public void update() {}

    public void quickTurnOnOff() {
        onFORCED();
        offFORCED();
    }

    public void onFORCED() {
        leftServo.servo[0].setPower(0.1);
        rightServo.servo[0].setPower(0.1);
    }

    public void offFORCED() {
        leftServo.servo[0].setPower(0.0);
        rightServo.servo[0].setPower(0.0);
    }

    public void on() {
        leftServo.setTargetPower(1.0);
        rightServo.setTargetPower(1.0);
    }

    public void reverse() {
        leftServo.setTargetPower(-1.0);
        rightServo.setTargetPower(-1.0);
    }

    public void off() {
        leftServo.setTargetPower(0.0);
        rightServo.setTargetPower(0.0);
    }

    public void leftReverse() {
        leftServo.setTargetPower(-1.0);
    }

    public void rightReverse() {
        rightServo.setTargetPower(-1.0);
    }

    public void leftUp() {
        leftServo.setTargetPower(1.0);
    }

    public void rightUp() {
        rightServo.setTargetPower(1.0);
    }
}
