package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;

@TeleOp
@Config
public class HangTester extends LinearOpMode {

    public static double fix = 1;
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo leftHang = hardwareMap.get(CRServo.class, "leftHang");
        CRServo rightHang = hardwareMap.get(CRServo.class, "rightHang");
        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot robot = new Robot(hardwareMap);
        PriorityCRServo hang = new PriorityCRServo(new CRServo[] {leftHang, rightHang}, "hang", 1, 2);
        robot.hardwareQueue.addDevice(hang);

        waitForStart();
        leftHang.setPower(1*fix);
        rightHang.setPower(1*fix);
        while (!isStopRequested()) {
        }
        //robot.hardwareQueue.update();
        if (isStopRequested()) {
            leftHang.setPower(0);
            rightHang.setPower(0);
        }
    }
}
