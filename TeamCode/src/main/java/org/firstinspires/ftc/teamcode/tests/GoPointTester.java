package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class GoPointTester extends LinearOpMode {
    public static double x = 20;
    public static double y = 32;
    public static double h = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        waitForStart();
        while (!isStopRequested()) {
            drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(h)));
            Canvas ctx = TelemetryUtil.packet.fieldOverlay();
            ctx.setStroke("red");
            ctx.strokeCircle(x, y, 3);
            robot.update();
        }
    }
}
