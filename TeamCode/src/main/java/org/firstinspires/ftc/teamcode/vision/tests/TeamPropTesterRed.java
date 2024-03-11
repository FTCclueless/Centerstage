package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

@Disabled
@TeleOp
public class TeamPropTesterRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap, telemetry, true, true, false);
        TeamPropDetectionPipeline teamPropDetectionPipeline;
        Robot robot = new Robot(hardwareMap, vision);

        teamPropDetectionPipeline = vision.teamPropDetectionPipeline;

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
