package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
public class LocalizationCompTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        robot.drivetrain.setPoseEstimate(new Pose2d(0.0, 0.0, Math.toRadians(0.0)));
        while(opModeInInit()){
            robot.update();
        }

        waitForStart();

        robot.goToPoint(new Pose2d(12.0, 0.0, Math.toRadians(0.0)), this, false, false, 0.75);

        //3 back and forth
        for(int i = 0; i < 3; i++) {
            robot.goToPoint(new Pose2d(84.0, 0.0, Math.toRadians(0.0)), this, false, false, 0.75);
            robot.goToPoint(new Pose2d(12.0, 0.0, Math.toRadians(0.0)), this, false, false, 0.75);
        }

        //3 strafing
        robot.goToPoint(new Pose2d(0.0, 0.0, Math.toRadians(90.0)), this, false, false, 0.75);
        for(int i = 0; i < 3; i++){
            robot.goToPoint(new Pose2d(84.0, 0.0, Math.toRadians(90.0)), this, false, false, 0.75);
            robot.goToPoint(new Pose2d(12.0, 0.0, Math.toRadians(90.0)), this, false, false, 0.75);
        }

        //3 diagonal
        robot.goToPoint(new Pose2d(12.0, 0.0, Math.toRadians(45.0)), this, false, false, 0.75);
        for(int i = 0; i < 3; i++){
            robot.goToPoint(new Pose2d(84.0, 0.0, Math.toRadians(45.0)), this, false, false, 0.75);
            robot.goToPoint(new Pose2d(12.0, 0.0, Math.toRadians(45.0)), this, false, false, 0.75);
        }

        robot.goToPoint(new Pose2d(0.0, 0.0, Math.toRadians(0.0)), this, true, true, 0.75);

        robot.drivetrain.state = Drivetrain.State.IDLE;

        while (!isStopRequested()){
            robot.update();
        }

    }
}
