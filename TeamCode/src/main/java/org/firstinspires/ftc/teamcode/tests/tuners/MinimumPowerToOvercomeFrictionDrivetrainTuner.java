package org.firstinspires.ftc.teamcode.tests.tuners;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.BNOLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.IMUMergeLocalizer;
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

    double[] sums = new double[4];
    int iterations = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;
        Localizer localizer = new IMUMergeLocalizer(hardwareMap, sensors, robot.drivetrain, "ff00ff", "ffff00");
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

            for (int a = 0; a < iterations; a++) {
                localizer.setPoseEstimate(new Pose2d(0, 0, 0));
                long start = System.currentTimeMillis();
                for (double j = 0; j < 1; j = (double) (System.currentTimeMillis() - start) / (15000.0)) {
                    Globals.START_LOOP();
                    robot.update();
                    TelemetryUtil.sendTelemetry();

                    motors.get(i).setTargetPower(j);

                    robotPose = robot.drivetrain.localizers[0].getPoseEstimate();
                    if (Math.abs(robotPose.x) > 0.1 || Math.abs(robotPose.y) > 0.1 || Math.abs(robotPose.heading) > Math.toRadians(1)) {
                        minPowersToOvercomeFriction[i] = j;
                        break;
                    }
                    telemetry.addData(motors.get(i).name + " current power: ", j);
                    telemetry.update();
                }

                motors.get(i).setTargetPower(0.0);

                sums[i] += minPowersToOvercomeFriction[i] * (12/sensors.getVoltage());

                long waitStart = System.currentTimeMillis();
                while (System.currentTimeMillis() - waitStart < 1000) {
                    robot.update();
                }
            }

            Log.e(motors.get(i).name + " AVERAGE min power with voltage correction", sums[i]/iterations + "");

        }
    }
}
