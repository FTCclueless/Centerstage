package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@TeleOp
public class TrackWidthTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Sensors sensors = robot.sensors;

        double ticksToInches = 0.000528438;

        int[] odometry;
        double theta;

        waitForStart();

        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);
            odometry = sensors.getOdometry();
            theta = Math.PI * 20; // 10 rotations

            robot.update();

            telemetry.addData("leftOdoRadius", (odometry[0]*ticksToInches)/theta + "");
            telemetry.addData("rightOdoRadius", (odometry[1]*ticksToInches)/theta + "");
            telemetry.update();
        }
    }
}
