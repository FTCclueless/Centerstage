package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp
public class AxonServoTuner extends LinearOpMode {
    public static String servoName = "bottomTurret";
    public static String encoderName = "analogInput2";
    public static double testPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, encoderName);
        TelemetryUtil.setup();

        double minAngle = 0;
        double maxAngle = 0;

        waitForStart();

        servo.setPosition(0);
        waitUntilServoDone(encoder);

        System.out.println("Servo is at base position");

        servo.setPosition(0);
        waitUntilServoDone(encoder);
        minAngle = getEncAngle(encoder);
        RobotLog.e("minAngle " + minAngle);

        servo.setPosition(1);
        waitUntilServoDone(encoder);
        maxAngle = getEncAngle(encoder);
        RobotLog.e("maxAngle " + maxAngle);

        /*while (opModeIsActive()) {
            servo.setPosition(testPos);
            TelemetryUtil.packet.put("enc", getEncAngle(encoder));
            TelemetryUtil.sendTelemetry();
        }*/

        servo.setPosition(0);
        waitUntilServoDone(encoder);

        servo.setPosition(1);
        long startTime = System.currentTimeMillis();
        waitUntilServoDone(encoder);
        long endTime = System.currentTimeMillis(); // Tee hee amirite
        servo.setPosition(0);

        RobotLog.e("posToRadians " + 1 / (maxAngle - minAngle));
        RobotLog.e("1: " + (maxAngle - minAngle) + " 2: " + (endTime - startTime));
        RobotLog.e("speed " + (maxAngle - minAngle) / ((endTime - startTime) / 1000));
    }

    public static double getEncAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }

    public static void waitUntilServoDone(AnalogInput encoder) {
        double lastAngle = getEncAngle(encoder);

        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 300) {
            double angle = getEncAngle(encoder);

            if (Math.abs(lastAngle - angle) > Math.toRadians(4)) {
                startTime = System.currentTimeMillis();
            }
            lastAngle = angle;
        }
    }
}
