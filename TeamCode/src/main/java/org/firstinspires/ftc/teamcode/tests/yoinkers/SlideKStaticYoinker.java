package org.firstinspires.ftc.teamcode.tests.yoinkers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
@TeleOp
public class SlideKStaticYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Slides slides = robot.deposit.slides;

        waitForStart();

        int count = 0;
        while (robot.sensors.getSlidesVelocity() == 0) {
            ((PriorityMotor)robot.hardwareQueue.getDevice("slidesMotor")).setTargetPower(count*0.005);
        }

        Log.e(count+ "", "e");

    }
}
