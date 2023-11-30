package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@Config
@TeleOp
public class AxonServoTuner extends LinearOpMode {
    public static String servoName = "testServo";
    public static String encoderName = "testEncoder";

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, encoderName);

        double minAngle = 0;
        double maxAngle = 0;

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(0);
            waitUntilServoDone(encoder);
            minAngle = getEncAngle(encoder);
            RobotLog.e("minAngle", minAngle);

            servo.setPosition(1);
            waitUntilServoDone(encoder);
            maxAngle = getEncAngle(encoder);
            RobotLog.e("maxAngle", maxAngle);

            servo.setPosition(0);
            waitUntilServoDone(encoder);

            servo.setPosition(1);
            long startTime = System.currentTimeMillis();
            while (getEncAngle(encoder) != maxAngle) {}
            long endTime = System.currentTimeMillis();

            RobotLog.e("radiansToPos", 1 / (maxAngle - minAngle));
            RobotLog.e("speed", (maxAngle - minAngle) / ((endTime - startTime) / 1000));
        }
    }

    public static double getEncAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }

    public static void waitUntilServoDone(AnalogInput encoder) {
        double lastAngle = getEncAngle(encoder);

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 100) {
            double angle = getEncAngle(encoder);

            if (lastAngle != angle) {
                startTime = System.currentTimeMillis();
                lastAngle = angle;
            }
        }
    }
}
