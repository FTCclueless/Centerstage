package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@Disabled
@TeleOp
public class EncoderDirectionTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            robot.update();

            telemetry.addData("right Encoder",drive.localizers[0].encoders[0].getCurrentDist());
            telemetry.addData("left Encoder",drive.localizers[0].encoders[1].getCurrentDist());
            telemetry.addData("back Encoder",drive.localizers[0].encoders[2].getCurrentDist());
            telemetry.update();
        }
    }
}
