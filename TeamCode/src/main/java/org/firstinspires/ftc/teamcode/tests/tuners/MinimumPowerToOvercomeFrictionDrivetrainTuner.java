package org.firstinspires.ftc.teamcode.tests.tuners;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;

@Autonomous
public class MinimumPowerToOvercomeFrictionDrivetrainTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Localizer localizer = robot.drivetrain.localizer;
        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<PriorityMotor> motors = new ArrayList<>();

        double[] minPowersToOvercomeFriction = new double[4];

        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof PriorityMotor) {
                motors.add((PriorityMotor) device);
            }
        }

        Pose2d robotPose;
        robot.drivetrain.resetMinPowersToOvercomeFriction();

        waitForStart();

        for (int i = 0; i < 4; i++) {
            localizer.setPoseEstimate(new Pose2d(0,0,0));

            for (double j = 0; j < 1; j += 0.00001) {
                Globals.START_LOOP();
                robot.drivetrain.update();
                robot.hardwareQueue.update();
                TelemetryUtil.sendTelemetry();

                motors.get(i).setTargetPower(j);

                robotPose = robot.drivetrain.localizer.getPoseEstimate();
                if (robotPose.x > 0.1 || robotPose.x < -0.1 || robotPose.y > 0.1 || robotPose.y < -0.1 || robotPose.heading > Math.toRadians(1) || robotPose.heading < Math.toRadians(-1)) {
                    minPowersToOvercomeFriction[i] = j;
                    break;
                }
                Log.e(motors.get(i).name + " ", "currnet power: " + j);

                telemetry.addData(motors.get(i).name + " current power: ", j);
                telemetry.update();
            }

            motors.get(i).setTargetPower(0.0);
            Thread.sleep(1000);

            Log.e(motors.get(i).name + " min power", minPowersToOvercomeFriction[i] + "");

            telemetry.addData(motors.get(i).name + " min power: ", minPowersToOvercomeFriction[i]);
            telemetry.update();
        }
    }
}
