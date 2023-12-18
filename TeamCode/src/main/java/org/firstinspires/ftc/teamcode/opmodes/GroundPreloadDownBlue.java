package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.pipelines.TeamPropDetectionPipeline;

@Autonomous
public class GroundPreloadDownBlue extends DoublePreloadAuto {
    public GroundPreloadDownBlue() {
        super();
        up = true;
        red = true;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        doInitialization();
        waitForStart();
        doGroundPreload();
    }
}
