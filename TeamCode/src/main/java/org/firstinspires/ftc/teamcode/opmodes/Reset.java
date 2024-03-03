package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.EndAffector;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;

        Robot robot = new Robot(hardwareMap);
        Deposit deposit = robot.deposit;
        EndAffector endAffector = robot.deposit.endAffector;

        robot.deposit.state = Deposit.State.IDLE;

        robot.hang.quickTurnOnOff();

        waitForStart();

        while (!isStopRequested()) {
            robot.drivetrain.setPoseEstimate(new Pose2d(0,0,0));
            endAffector.v4Servo.setTargetAngle(Deposit.v4BarTransferAngle, 0.5);
            endAffector.topServo.setTargetAngle(Deposit.topServoTransferAngle,1.0);
            deposit.release.preGrab();

            robot.update();
        }
    }
}
