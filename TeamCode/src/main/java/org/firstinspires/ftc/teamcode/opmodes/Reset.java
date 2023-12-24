package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.EndAffector;

@TeleOp
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Deposit deposit = robot.deposit;
        EndAffector endAffector = robot.deposit.endAffector;

        robot.deposit.state = Deposit.State.IDLE;

        waitForStart();

        while (!isStopRequested()) {
            endAffector.v4Servo.setTargetAngle(Deposit.v4BarIntakeAngle, 0.5);
            endAffector.topServo.setTargetAngle(Deposit.topServoIntakeAngle,1.0);
            deposit.release.hold();

            robot.update();
        }
    }
}
