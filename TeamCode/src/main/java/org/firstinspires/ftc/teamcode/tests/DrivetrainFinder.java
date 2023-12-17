package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Disabled
@Config
@TeleOp
public class DrivetrainFinder extends LinearOpMode {
    public static double target = 0.5;
      Sensors sensors;
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        sensors = robot.sensors;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            ((PriorityMotor)robot.hardwareQueue.getDevice("rightRear")).setTargetPower(target);
            Log.e("targetPow", "" + ((PriorityMotor)robot.hardwareQueue.getDevice("leftRear")).getPower());

            TelemetryUtil.packet.put("pow", ((PriorityMotor)robot.hardwareQueue.getDevice("leftRear")).motor[0].getPower());
        }
    }
}
