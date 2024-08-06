package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap, telemetry, false, false, false);
        Robot robot = new Robot(hardwareMap, vision);
        Sensors sensors = robot.sensors;

        Globals.RUNMODE = RunMode.TESTER;
        ButtonToggle bty = new ButtonToggle();

        waitForStart();

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        SparkFunOTOS.Pose2D lastPose;
        SparkFunOTOS.Pose2D currPose = new SparkFunOTOS.Pose2D(0,0,0);

        File file = AppUtil.getInstance().getSettingsFile("deceldata.csv");
        FileWriter fw;
        try {
            fw = new FileWriter(file);
        } catch (IOException e) {
            System.out.println("BAD BAD BAD BAD BAD");
            return;
        }

        while(!isStopRequested()) {
            lastPose = currPose;
            robot.drivetrain.drive(gamepad1);
            TelemetryUtil.packet.put("leftOdo", sensors.getOdometry()[0]);
            TelemetryUtil.packet.put("rightOdo", sensors.getOdometry()[1]);
            TelemetryUtil.packet.put("backOdo", sensors.getOdometry()[2]);
            robot.update();
            double changeX = currPose.x - lastPose.x;
            double changeY = currPose.y - lastPose.y;
            double chnageH = currPose.h - lastPose.h;

            double relChangeX = changeX * Math.cos(lastPose.h) + changeY * Math.sin(lastPose.h);
            double relChangeY = changeX * -Math.sin(lastPose.h) + changeY * Math.cos(lastPose.h);

            String buffer = "";


            buffer += robot.drivetrain.localizers[0].relDeltaX + "," + relChangeX + "," + robot.drivetrain.localizers[0].relDeltaY + "," + relChangeY + Globals.LOOP_TIME + "," + Math.toDegrees(chnageH) + "\n";

            try {
                fw.write(buffer);
            } catch (IOException e) {
                System.out.println("bad :(");
                return;
            }
            if (bty.isClicked(gamepad1.y)) {
                try {
                    fw.flush();
                } catch (IOException e) {
                    System.out.println("BAD BAD BAD BAD BAD");
                }
            }
        }

    }
}
