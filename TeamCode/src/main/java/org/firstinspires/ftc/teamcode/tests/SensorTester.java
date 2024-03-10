package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Disabled
@TeleOp
public class SensorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;

        Robot robot = new Robot(hardwareMap);
        robot.deposit.state = Deposit.State.IDLE;
        Sensors sensors = robot.sensors;

        waitForStart();

        while (!isStopRequested()) {
            robot.sensors.update();

            telemetry.addData("leftOdo", sensors.getOdometry()[0]);
            telemetry.addData("rightOdo", sensors.getOdometry()[1]);
            telemetry.addData("backOdo", sensors.getOdometry()[2]);

            telemetry.addData("slides encoder", sensors.getSlidesPos());

            telemetry.addData("intake_triggered", sensors.isIntakeTriggered());
            telemetry.addData("deposit_triggered", sensors.isDepositTriggered());

            telemetry.addData("board_ir", sensors.getBoardIR());

            telemetry.addData("intakeColorSensorDist", robot.intake.forcePullColorSensorDist());

            telemetry.addData("cornerLeftUltrasonic", sensors.getCornerLeftDist());
            telemetry.addData("cornerRightUltrasonic", sensors.getCornerRightDist());
            telemetry.addData("frontUltrasonic", sensors.getFrontDist());

            telemetry.update();
        }
    }
}
