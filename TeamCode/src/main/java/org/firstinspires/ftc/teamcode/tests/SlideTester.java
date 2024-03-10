package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;


@Disabled
@TeleOp
@Config
public class SlideTester extends LinearOpMode {
    public static double distance = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Slides slides = robot.deposit.slides;
        robot.deposit.state = Deposit.State.IDLE;
        waitForStart();

        while (!isStopRequested()) {
            Globals.START_LOOP();
            slides.setTargetLength(distance);
            TelemetryUtil.packet.put("state", robot.deposit.state);

            robot.sensors.update();
            robot.deposit.update();
            TelemetryUtil.sendTelemetry();
            robot.hardwareQueue.update();
        }
    }
}
