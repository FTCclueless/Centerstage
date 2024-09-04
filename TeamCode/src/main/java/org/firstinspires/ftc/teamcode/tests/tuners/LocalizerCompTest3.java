package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class LocalizerCompTest3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        //left and right 60, 40 forward, diamond shape
        robot.goToPoint(new Pose2d(20, 60, Math.PI), this);
        robot.goToPoint(new Pose2d(40, 0, -Math.PI/2), this);
        robot.goToPoint(new Pose2d(20, -60, Math.PI/2), this);
        robot.goToPoint(new Pose2d(0, 0, 0), this);

        while (!isStopRequested()){
            robot.update();
        }

    }
}
