package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@TeleOp
public class EncoderPoseTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Sensors sensors = robot.sensors;

        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.984252; //for 50mm wheels
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;

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
            telemetry.addData("backOdoRadius", (odometry[2]*ticksToInches)/theta + "");
            telemetry.update();
        }
    }
}
