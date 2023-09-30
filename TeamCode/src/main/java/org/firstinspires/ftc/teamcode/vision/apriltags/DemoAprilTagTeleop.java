package org.firstinspires.ftc.teamcode.vision.apriltags;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class DemoAprilTagTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagLocalizer vision = new AprilTagLocalizer(hardwareMap);
        TelemetryUtil.setup();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            vision.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
