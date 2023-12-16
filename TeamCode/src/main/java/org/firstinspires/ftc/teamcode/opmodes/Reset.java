package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.EndAffector;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

@TeleOp
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Deposit deposit = robot.deposit;
        EndAffector endAffector = robot.deposit.endAffector;

        robot.deposit.state = Deposit.State.WAIT;

        waitForStart();

        while (!isStopRequested()) {
            endAffector.v4Servo.setTargetAngle(Deposit.downPitch, 1.0);
            endAffector.topTurret.setTargetAngle(Deposit.intakeTopTurret,1.0);
            endAffector.botTurret.setTargetAngle(Deposit.intakeBotTurret,1.0);
            endAffector.topServo.setTargetAngle(Deposit.intakeTopServoAngle,1.0);
            deposit.dunker.intake();

            robot.update();
        }
    }
}
