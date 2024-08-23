package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

public class LocalizerCompTest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.goToPoint(new Pose2d(40, 0, 0), this);
        robot.goToPoint(new Pose2d(40, 120, 0), this);
        robot.goToPoint(new Pose2d(0, 120, 0), this);
        robot.goToPoint(new Pose2d(0, 0, 0), this);

        while (!isStopRequested()){
            robot.update();
        }

    }
}
