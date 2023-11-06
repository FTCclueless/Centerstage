package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class Teleop extends LinearOpMode {
    enum State {
        READY,
        INTAKE,
        DEPOSIT,
        DUNK;

        public static State nextState(State state) {
            switch (state) {
                case READY:
                    return INTAKE;
                case INTAKE:
                    return DEPOSIT;
                case DEPOSIT:
                    return DUNK;
                case DUNK:
                    return READY;
            }
            return READY;
        }
    };

    private State state = State.READY;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.TELEOP;
        ButtonToggle btnX =      new ButtonToggle();
        ButtonToggle btnB =      new ButtonToggle();
        ButtonToggle btnA =      new ButtonToggle();
        ButtonToggle btnY =      new ButtonToggle();
        ButtonToggle dpadUp =    new ButtonToggle();
        ButtonToggle dpadDown =  new ButtonToggle();
        ButtonToggle dpadLeft =  new ButtonToggle();
        ButtonToggle dpadRight = new ButtonToggle();

        waitForStart();

        while (!isStopRequested()) {
            switch (state) {
                case READY:
                    break;

                case INTAKE:
                    if (btnB.isToggled(gamepad1.b)) {
                        robot.intake.on();
                    } else {
                        robot.intake.off();
                    }

                    if (gamepad1.a) { // TODO: Make a ButtonToggle
                        robot.intake.setActuationPixelHeight(5);
                    }
                    if (gamepad1.y) {
                        robot.intake.setActuationPixelHeight(1);
                    }
                    break;

                case DEPOSIT:
                    // use depositAt in order to get the thing done

                    //if (dpadUp.isClicked(gamepad1.dpad_up))
                    //if (dpadDown.isClicked(gamepad1.dpad_down)
                    //if (dpadLeft.isClicked(gamepad1.dpad_left)
                    //if (dpadRight.isClicked(gamepad1.dpad_right)

                    if (gamepad1.a) {
                        // Offset forward
                    }
                    if (gamepad1.y) {
                        // Offset backward
                    }

                    break;

                case DUNK:
                    // if (btnA.isClicked(gamepad1.a)) dunk

                    if (/* pixel count 0 */ false)
                        // Reset deposit offset
                        state = State.READY;

                    break;
            }

            robot.drivetrain.drive(gamepad1);
            if (btnX.isClicked(gamepad1.x)) // This might brick during DUNK to ready -- Eric
                state = State.nextState(state);

            robot.update();
        }
    }
}
