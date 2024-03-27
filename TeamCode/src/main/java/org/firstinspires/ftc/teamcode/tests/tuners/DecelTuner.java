package org.firstinspires.ftc.teamcode.tests.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class DecelTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Pose2d max = new Pose2d(0, 0, 0);
        double maxMag = 0;
        Pose2d min = new Pose2d(0, 0, 0);
        double minMag = 0;

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);
            Pose2d acc = robot.drivetrain.localizer.relCurrentAcc;
            max.x = Math.max(acc.x, max.x);
            max.y = Math.max(acc.y, max.y);
            max.heading = Math.max(acc.heading, max.heading);
            maxMag = Math.sqrt(max.x * max.x + max.y * max.y);
            min.x = Math.min(acc.x, min.x);
            min.y = Math.min(acc.y, min.y);
            min.heading = Math.min(acc.heading, min.heading);
            minMag = Math.sqrt(min.x * min.x + min.y * min.y);

            TelemetryUtil.packet.put("max.x", max.x);
            TelemetryUtil.packet.put("max.y", max.y);
            TelemetryUtil.packet.put("max.heading", max.heading);
            TelemetryUtil.packet.put("maxMag", maxMag);
            TelemetryUtil.packet.put("min.x", min.x);
            TelemetryUtil.packet.put("min.y", min.y);
            TelemetryUtil.packet.put("min.heading", min.heading);
            TelemetryUtil.packet.put("minMag", minMag);
            TelemetryUtil.packet.put("acc.x", acc.x);
            TelemetryUtil.packet.put("acc.y", acc.y);
            TelemetryUtil.packet.put("acc.heading", acc.heading);
            robot.update();
        }
    }
}
