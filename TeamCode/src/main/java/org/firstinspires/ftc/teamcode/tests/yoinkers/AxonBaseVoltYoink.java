package org.firstinspires.ftc.teamcode.tests.yoinkers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@Config
@TeleOp
public class AxonBaseVoltYoink extends LinearOpMode {
    Robot robot;
    public static String servoName = "bottomTurret";
    public static String encoderName = "analogInput2";
    public static double basePos = 0;
    public static double testPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.deposit.state = Deposit.State.IDLE;
        robot.update();
        robot.deposit.endAffector.v4Servo.setTargetAngle(0,1);
        robot.hardwareQueue.update();
        Servo servo = hardwareMap.get(Servo.class, servoName);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, encoderName);
        TelemetryUtil.setup();

        double minAngle = 0;
        double maxAngle = 0;

        waitForStart();

        servo.setPosition(0);
        waitUntilServoDone(encoder);

        System.out.println("Servo is at base position");

        servo.setPosition(basePos);
        waitUntilServoDone(encoder);
        Log.e("Voltage", encoder.getVoltage() + "");
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
