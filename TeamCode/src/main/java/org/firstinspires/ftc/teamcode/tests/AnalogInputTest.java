package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@TeleOp
@Config
public class AnalogInputTest extends LinearOpMode {
    public static String encoderName = "analogInput0";

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.TESTER;
        robot.deposit.state = Deposit.State.INTAKE;
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, encoderName);
        TelemetryUtil.setup();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryUtil.packet.put("encoderValue", encoder.getVoltage() / 3.3 * 180);
            robot.update();
        }
    }
}
