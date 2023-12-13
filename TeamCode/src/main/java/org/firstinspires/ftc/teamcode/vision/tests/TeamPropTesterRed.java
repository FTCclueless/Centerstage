package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

@TeleOp
public class TeamPropTesterRed extends LinearOpMode {
    private Vision vision = new Vision(hardwareMap, telemetry, true, true, false);
    private TeamPropDetectionPipeline teamPropDetectionPipeline;
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, vision);

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
