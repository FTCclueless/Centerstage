package org.firstinspires.ftc.teamcode.tests.yoinkers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
@TeleOp
public class SlideKStaticYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Slides slides = robot.deposit.slides;

        waitForStart();

        double power = 0;

        // making sure slides are down
        double lastDist = 0.0;
        double vel = 0.0;
        long start = System.currentTimeMillis();
        long velocityStart = System.currentTimeMillis();
        robot.deposit.slides.manualMode = true;

        while (vel < 1 && opModeIsActive()) {
            if ((System.currentTimeMillis() - velocityStart) > 100) {
                vel = Math.abs((robot.deposit.slides.getLength()-lastDist)/0.1);
                lastDist = robot.deposit.slides.getLength();
                velocityStart = System.currentTimeMillis();
            }

            power += 0.001;
            ((PriorityMotor)robot.hardwareQueue.getDevice("slidesMotor")).motor[0].setPower(-power);
            ((PriorityMotor)robot.hardwareQueue.getDevice("slidesMotor")).motor[1].setPower(-power);
            TelemetryUtil.packet.put("power", power);
            TelemetryUtil.packet.put("velo", robot.sensors.getSlidesVelocity());
            robot.sensors.update();
            robot.deposit.slides.update();
            TelemetryUtil.sendTelemetry();
            Log.e("vel", vel + "");
            Log.e("power", power + "");
        }

        Log.e(power+ "", "e");

    }
}
