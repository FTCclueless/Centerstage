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
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Sensors sensors = robot.sensors;

        double ticksPerRotation = 2000.0;
        double wheelRadius = 0.62992126; //for 50mm wheels
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;

        Encoder[] encoders = new Encoder[3];

        encoders[0] = new Encoder(new Pose2d(0,6.029007875523001 * 3602.392103039499 / 3600.00 * 3607.209152475203 / 3600.00 * 3601.3849017938674 / 3600.0 * 3601.0674144447016 / 3600.0 * 3601.0674144447016 / 3600.0 * 3602.0 / 3600.0),  1); // left
        encoders[1] = new Encoder(new Pose2d(0,-5.795874017197001 * 3602.392103039499 / 3600.00 * 3607.209152475203 / 3600.00 * 3601.3849017938674 / 3600.0 * 3601.0674144447016 / 3600.0 * 3601.0674144447016 / 3600.0 * 3602.0 / 3600.0),1); // right
        encoders[2] = new Encoder(new Pose2d(-10.140440947417002, 0),  1); // back

        int[]odometry;
        double theta;

        waitForStart();



        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);
            odometry = sensors.getOdometry();

            for (int i = 0; i < odometry.length; i ++){
                encoders[i].update(odometry[i]);
            }

            theta = Math.PI * 20; // 10 rotations

            robot.update();

            telemetry.addData("leftOdoRadius", encoders[0].getCurrentDist()/theta + "");
            telemetry.addData("rightOdoRadius", encoders[1].getCurrentDist()/theta + "");
            telemetry.addData("backOdoRadius", encoders[2].getCurrentDist()/theta + "");
            telemetry.update();
        }
    }
}
