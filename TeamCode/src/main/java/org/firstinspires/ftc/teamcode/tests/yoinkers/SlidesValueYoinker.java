package org.firstinspires.ftc.teamcode.tests.yoinkers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Disabled
@Config
@TeleOp(group = "tests")
public class SlidesValueYoinker extends LinearOpMode { //this is probably useless --Kyle
    @Override
    public void runOpMode() {
        /*Robot robot = new Robot(hardwareMap);
        Slides slides = new Slides(hardwareMap, robot.hardwareQueue, robot.sensors);

        waitForStart();
        double pow = 0.02;
        int counter = 0;
        double maxVel;
        while (pow <= 1) {
            maxVel = 0;
            slides.update();
            boolean done = false;
            if (!done) {
                ((PriorityMotor) robot.hardwareQueue.getDevice("slidesMotor")).setTargetPower(pow);
                double vel = slides.vel;
                if (vel > maxVel) {
                    maxVel = vel;
                }
                if (Math.abs(slides.length-slides.maxSlidesHeight) < 1) {
                    Log.e("pow: " + pow, "vel: " + maxVel);
                    done = true;
                }
            }
            else if (Math.abs(slides.length) < 1) {

                done = false;
                counter ++;
                if (counter == 3) {
                    pow +=0.02;
                }
            }

        }*/
    }
}
