package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@TeleOp
public class SensorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        Globals.RUNMODE = RunMode.TESTER;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();

            telemetry.addData("leftOdo", sensors.getOdometry()[0]);
            telemetry.addData("rightOdo", sensors.getOdometry()[1]);
            telemetry.addData("backOdo", sensors.getOdometry()[2]);

            telemetry.addData("slides encoder", sensors.getSlidesPos());

            telemetry.addData("intake_triggered", sensors.isIntakeTriggered());
            telemetry.addData("deposit_triggered", sensors.isDepositTriggered());

            telemetry.addData("dist_left", sensors.getDistLeft());
            telemetry.addData("dist_right", sensors.getDistRight());

            telemetry.update();
        }
    }
}
