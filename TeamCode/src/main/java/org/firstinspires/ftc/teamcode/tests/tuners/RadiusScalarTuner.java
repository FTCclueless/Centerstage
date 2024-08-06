package org.firstinspires.ftc.teamcode.tests.tuners;

import static org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain.centripetalTune;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

// 1.9561633497167892 value, fwd 0.2, radius 20
// 1.1462466583941862 value, fwd 0.4, radius 20
// 0.7251067454381004 value, fwd 0.6, radius 20
// 0.5391677195025867 value, fwd 0.8, radius 20
// 0.5994616688147713 value, fwd 1.0, radius 20

@TeleOp(group = "tests", name = "Radius Scalar Tuner")
@Config
public class RadiusScalarTuner extends LinearOpMode {
    public static double targetFwd = 0.8;
    public static double radius = 20;
    public static double value = 1.6;
    public static double errorThreshold = 0.05;

    // 1.878637426244083 for 0.3 power @ radius 20

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        double error = 1;

        waitForStart();
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        long startTime = System.currentTimeMillis();
        targetFwd /= 2;
        while (System.currentTimeMillis() < startTime + 250 && opModeIsActive()) {
            double targetTurn = (value * (TRACK_WIDTH / radius)) * targetFwd;
        //double centripetal = centripetalTune*targetFwd*targetFwd/radius;
            double[] motorPowers = {
                targetFwd - targetTurn,// - centripetal,
                targetFwd - targetTurn,// + centripetal,
                targetFwd + targetTurn,// - centripetal,
                targetFwd + targetTurn// + centripetal,
            };
            robot.drivetrain.normalizeArray(motorPowers);


            robot.drivetrain.state = Drivetrain.State.IDLE;
            robot.drivetrain.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);

            robot.update();
        }
        targetFwd *= 2;

        do {
            startTime = System.currentTimeMillis();

            double sumFwd = 0;
            double sumTheta = 0;

            while (System.currentTimeMillis() < startTime + 500) {
                double targetTurn = (value * (TRACK_WIDTH / radius)) * targetFwd;
                double centripetal = 0.5*Math.pow(robot.drivetrain.localizers[0].getRelativePoseVelocity().x / Globals.MAX_X_SPEED, 2)/radius;
                double[] motorPowers = new double[] {
                    targetFwd - targetTurn - centripetal,
                    targetFwd - targetTurn + centripetal,
                    targetFwd + targetTurn - centripetal,
                    targetFwd + targetTurn + centripetal,
                };
                robot.drivetrain.normalizeArray(motorPowers);


                robot.drivetrain.state = Drivetrain.State.IDLE;
                robot.drivetrain.setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);

                double leftEnc = robot.drivetrain.localizers[0].encoders[0].getDelta();
                double rightEnc = robot.drivetrain.localizers[0].encoders[1].getDelta();
                double leftY = robot.drivetrain.localizers[0].encoders[0].y;
                double rightY = robot.drivetrain.localizers[0].encoders[1].y;

                double fwd = (rightEnc * leftY - leftEnc * rightY) / (leftY - rightY);
                double theta = (rightEnc - leftEnc) / (leftY - rightY);
                sumFwd += fwd;
                sumTheta += theta;
                double foundRadius = sumFwd / sumTheta;
                Log.e("LOOK AT ME FOUND RADIUS", foundRadius + "");
                error = (radius / (sumFwd / sumTheta));

                TelemetryUtil.packet.put("radiusError", error);
                TelemetryUtil.packet.put("radiusValue", value);

                robot.update();
            }

            value /= (9 + error) / 10;
            Log.e("LOOK AT ME ERROR", error + "");
            Log.e("LOOK AT ME VALUE", value + "");
        } while (opModeIsActive() && Math.abs(error - 1) > errorThreshold);
    }
}
