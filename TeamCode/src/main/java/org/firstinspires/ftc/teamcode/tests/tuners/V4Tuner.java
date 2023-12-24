package org.firstinspires.ftc.teamcode.tests.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Release;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@Config
@TeleOp
public class V4Tuner extends LinearOpMode {
    public static double pos = 0.4;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Deposit deposit = robot.deposit;
        Release servo = robot.deposit.release;

        waitForStart();
        while (!isStopRequested()) {
            servo.release.setTargetPose(pos, 1); //do this for v4 bar and 2 turrets to find base position, then find their required positions
            TelemetryUtil.packet.put("currentAngle", servo.release.getCurrentAngle());
            robot.update();
        }
    }
}
