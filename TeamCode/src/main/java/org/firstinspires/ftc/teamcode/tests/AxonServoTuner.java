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
    public static String servoName = "bottomTurret";
    public static String encoderName = "servoThing1";
    public static double speed = 0.002;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, encoderName);

        double minAngle = 0;
        double maxAngle = 0;
        double minPose = 0;
        double maxPose = 0;

        waitForStart();

        servo.setPosition(servo.getPosition());
        waitUntilServoDone(encoder);

        System.out.println("Servo is at base position");

        minPose = moveServoUntilCant(servo, encoder, 0);
        minAngle = getEncAngle(encoder);
        RobotLog.e("minAngle " + Math.toRadians(minAngle));

        maxPose = moveServoUntilCant(servo, encoder, 1);
        maxAngle = getEncAngle(encoder);
        RobotLog.e("maxAngle " + Math.toRadians(maxAngle));

        /*servo.setPosition(0);
        waitUntilServoDone(encoder);

        servo.setPosition(1);
        long startTime = System.currentTimeMillis();
        waitUntilServoDone(encoder);
        long endTime = System.currentTimeMillis() - 100; // Tee hee amirite
        servo.setPosition(0);*/

        RobotLog.e("posToRadians " + (maxAngle - minAngle) / (maxPose - minPose));
        //RobotLog.e("speed " + (maxAngle - minAngle) / ((endTime - startTime) / 1000));
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

    public static double moveServoUntilCant(Servo servo, AnalogInput encoder, double position) {
        double currentPos = servo.getPosition();
        double lastAngle = getEncAngle(encoder);
        boolean tickedOnce = false;

        if (currentPos == position)
            return position;

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 100 || !tickedOnce) {
            double angle = getEncAngle(encoder);

            if (lastAngle != angle) {
                startTime = System.currentTimeMillis();
                lastAngle = angle;
                tickedOnce = true;
            } else {
                currentPos = Math.min(currentPos + speed, position);
                servo.setPosition(currentPos);
            }
        }
        return currentPos;
    }
}
