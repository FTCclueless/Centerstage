package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class intakeCurrentTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        waitForStart();
        robot.intake.setActuationHeight(4);
        robot.intake.on();
        while (!isStopRequested()) {
            Log.e("current", "" + robot.sensors.getIntakeCurrent());
            TelemetryUtil.packet.put("current", robot.sensors.getIntakeCurrent());
            robot.update();
        }
    }
}
