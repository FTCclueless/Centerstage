package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.apriltags.AprilTagLocalizer;

@TeleOp
public class AprilTagTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Localizer localizer = new Localizer(hardwareMap, true);
        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, localizer);
        TelemetryUtil.setup();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            aprilTagLocalizer.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
