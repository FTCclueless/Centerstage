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
        while (Math.abs(robot.sensors.getSlidesVelocity()) <= 10 && opModeIsActive()) {
            power += 0.001;
            ((PriorityMotor)robot.hardwareQueue.getDevice("slidesMotor")).motor[0].setPower(-power);
            ((PriorityMotor)robot.hardwareQueue.getDevice("slidesMotor")).motor[1].setPower(-power);
            TelemetryUtil.packet.put("power", power);
            TelemetryUtil.packet.put("velo", robot.sensors.getSlidesVelocity());
            robot.sensors.update();
            TelemetryUtil.sendTelemetry();
            Log.e("robot.sensors.getSlidesVelocity()", robot.sensors.getSlidesVelocity() + "");
            Log.e("opModeIsActive", opModeIsActive() + "");
//            robot.hardwareQueue.update();
        }

        Log.e(power+ "", "e");

    }
}
