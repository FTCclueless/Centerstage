package org.firstinspires.ftc.teamcode.tests.tuners;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp
public class DecelTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        String buffer = "";
        boolean stopToggle = false;
        Pose2d start = new Pose2d(0, 0, 0);
        Pose2d vnaught = new Pose2d(0, 0, 0);
        Pose2d delta = new Pose2d(0, 0, 0);
        Pose2d expectedEnd = new Pose2d(0, 0 , 0);
        File file = AppUtil.getInstance().getSettingsFile("deceldata.csv");
        FileWriter fw;
        ButtonToggle bty = new ButtonToggle();
        try {
            fw = new FileWriter(file);
        } catch (IOException e) {
            System.out.println("BAD BAD BAD BAD BAD BAD");
            return;
        }

        waitForStart();

        while (opModeIsActive()) {
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;

            if (!stopToggle) {
                robot.drivetrain.drive(gamepad1);
            } else {
                gamepad1.rumble(1.0, 1.0, 100);
                robot.drivetrain.setMotorPowers(0, 0, 0, 0);

                if (robot.drivetrain.localizers[0].getRelativePoseVelocity().toVec3().getMag() <= 0.2) {
                    Vector3 end = robot.drivetrain.getPoseEstimate().toVec3();
                    delta = Vector3.subtract(end, start.toVec3()).toPose();
                    buffer += pose2dCSV(vnaught) + "," + pose2dCSV(delta) + "\n";
                    stopToggle = false;
                }
            }
            DashboardUtil.drawRobot(TelemetryUtil.packet.fieldOverlay(), expectedEnd, "#00FF00");

            if (x) {
                vnaught = robot.drivetrain.localizers[0].getRelativePoseVelocity();
                start = robot.drivetrain.localizers[0].getPoseEstimate();
                expectedEnd = robot.drivetrain.localizers[0].expected.clone();
                stopToggle = true;
            }
            if (bty.isToggled(y)) {
                try {
                    fw.write(buffer);
                    fw.flush();
                } catch (IOException e) {
                    System.out.println("BAD BAD BAD BAD BAD BAD");
                    return;
                }
                buffer = "";
            }
            robot.update();
        }

        try {
            fw.flush();
        } catch (IOException e) {
            System.out.println("BAD BAD BAD BAD BAD");
        }
    }

    public static String pose2dCSV(Pose2d pose2d) {
        return pose2d.x + "," + pose2d.y + "," + pose2d.heading;
    }
}
