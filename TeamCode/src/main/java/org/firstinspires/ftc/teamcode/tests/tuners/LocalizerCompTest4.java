package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class LocalizerCompTest4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        //
        robot.splineToPoint(new Pose2d(80, 20, Math.PI/2), this, false, false, false);
        robot.splineToPoint(new Pose2d(20, 40, Math.PI), this, false ,false, false);
        robot.splineToPoint(new Pose2d(0, 20, -Math.PI/2), this, false, false, false);
        robot.splineToPoint(new Pose2d(20, 0, 0), this, false, false, false);
        robot.splineToPoint(new Pose2d(40, 40, 0), this, false, false, false);
        robot.splineToPoint(new Pose2d(80, 0, 0), this, false, false, false);

        while (!isStopRequested()){
            robot.update();
        }

    }
}
