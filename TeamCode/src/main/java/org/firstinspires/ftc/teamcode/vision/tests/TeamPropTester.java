package org.firstinspires.ftc.teamcode.vision.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeamPropTester extends LinearOpMode {
    private VisionPortal visionPortal;
    private TeamPropDetectionPipeline teamPropDetectionPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(teamPropDetectionPipeline)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {} // waiting for camera to start streaming

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(6, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(250);

        while (opModeInInit()) {
            telemetry.addData("leftAvg", teamPropDetectionPipeline.leftAvg);
            telemetry.addData("centerAvg", teamPropDetectionPipeline.centerAvg);
            telemetry.addData("rightAvg", teamPropDetectionPipeline.rightAvg);
            telemetry.addData("propLocation: ", teamPropDetectionPipeline.getTeamPropLocation());
            telemetry.update();
        }

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {}
    }
}
