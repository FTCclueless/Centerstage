package org.firstinspires.ftc.teamcode.tests.yoinkers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Disabled
@TeleOp
public class SlidesTicksToInchesYoinker extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        DcMotorEx motor0 = ((PriorityMotor) robot.hardwareQueue.getDevice("slidesMotor")).motor[0];
        DcMotorEx motor1 = ((PriorityMotor) robot.hardwareQueue.getDevice("slidesMotor")).motor[1];

        waitForStart();
        long timeStart = System.currentTimeMillis();

        while (opModeIsActive()) {
            motor0.setPower(-1);
            motor1.setPower(-1);

            System.out.println(motor0.getVelocity());
            if (motor0.getVelocity() <= 5 && System.currentTimeMillis() - timeStart > 1000) {
                System.out.println("GAY PEOPLE " + motor0.getCurrentPosition());
                System.out.println("OOGA BOOGA LOG MASTA! " + Slides.maxSlidesHeight / motor0.getCurrentPosition());
                break;
            }
        }

        // Get max power

        timeStart = System.currentTimeMillis();
        double velocity = motor0.getVelocity() * Slides.ticksToInches;
        while (opModeIsActive()) {
            motor0.setPower(1);
            motor1.setPower(1);

            velocity = Math.max(velocity, motor0.getVelocity() * Slides.ticksToInches);

            if (motor0.getVelocity() <= 5 && System.currentTimeMillis() - timeStart > 1000) {
                System.out.println("Smelly shrek basllz " + velocity);
                break;
            }
        }
    }
}
