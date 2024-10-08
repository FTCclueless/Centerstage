package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
public class EncoderPoseTuner extends LinearOpMode {
    @Override

    private Encoder[] encoders;

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Sensors sensors = robot.sensors;

        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.984252; //for 50mm wheels
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;

        encoders = new Encoder[3];

        encoders[0] = new Encoder(new Pose2d(0,7.233361867),  -1); // left
        encoders[1] = new Encoder(new Pose2d(0,-6.7290861),-1); // right
        encoders[2] = new Encoder(new Pose2d(-9.018771197, 0),  -1); // back

        int[]odometry;
        double theta;

        waitForStart();



        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);
            odometry = sensors.getOdometry();
            updateEncoders(odometry);
            theta = Math.PI * 20; // 10 rotations

            robot.update();

            telemetry.addData("leftOdoRadius", encoders[0].getCurrentDist()/theta + "");
            telemetry.addData("rightOdoRadius", encoders[1].getCurrentDist()/theta + "");
            telemetry.addData("backOdoRadius", encoders[2].getCurrentDist()/theta + "");
            telemetry.update();
        }
    }
    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }
}
