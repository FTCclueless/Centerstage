package org.firstinspires.ftc.teamcode.tests.tuners;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@TeleOp(group = "tests", name = "Kinetic Kstatic Tuner")
@Config
public class KineticKStaticTuner extends LinearOpMode {
    public static double scalar = 0.0001;

    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.drivetrain.state = Drivetrain.State.IDLE;
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        /*robot.drivetrain.rightRear.setMinimumPowerToOvercomeFriction(0);
        robot.drivetrain.rightFront.setMinimumPowerToOvercomeFriction(0);
        robot.drivetrain.leftRear.setMinimumPowerToOvercomeFriction(0);
        robot.drivetrain.leftFront.setMinimumPowerToOvercomeFriction(0); ya use this later ndjasduias h*/

        waitForStart();

        double power = 0.0;

        while (opModeIsActive()) {
            double velx = robot.drivetrain.localizers[0].getRelativePoseVelocity().x;

            power += scalar * (velx > 5.0 ? -1 : 1);

            robot.drivetrain.setMoveVector(new Vector2(power, 0), 0);
            Log.e("LOOK AT ME", power + "");

            robot.update();
        }
    }
}
