package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class LocalizerCompTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.splineToPoint(new Pose2d(40, 60, 0), this, false, false, false);
        robot.splineToPoint(new Pose2d(20, 120, 0), this, false ,false, false);
        robot.splineToPoint(new Pose2d(0, 60, 0), this, false, false, false);
        robot.splineToPoint(new Pose2d(0, 0, 0), this, false, false, false);

        while (!isStopRequested()){
            robot.update();
        }

    }
}
