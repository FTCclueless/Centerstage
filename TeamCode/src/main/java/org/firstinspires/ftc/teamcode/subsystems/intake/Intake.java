package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Intake {
    enum State {
        ON,
        OFF,
        REVERSED
    }

    private final PriorityMotor intake;
    public State state = State.ON;
    private final Sensors sensors;
    public static double intakePower = 0.5; // CHANGE: Made this editable in FTC dashboard

    private boolean alreadyTriggered = false;
    private int numberOfTimesIntakeBeamBreakTriggered = 0;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2);

        hardwareQueue.addDevice(intake);
    }

    public void update() {
        if (sensors.isIntakeTriggered() && !alreadyTriggered) {
            alreadyTriggered = true;
            if (state == State.ON) {
                numberOfTimesIntakeBeamBreakTriggered++;
            } else {
                numberOfTimesIntakeBeamBreakTriggered--;
            }
        }
        if (!sensors.isIntakeTriggered()) {
            alreadyTriggered = false;
        }

        if (numberOfTimesIntakeBeamBreakTriggered > 2) {
            reverse();
        }

        // TODO: Might need to have a delay bc pixels may not have reached transfer - Huddy kim apparently
        switch (state) {
            case ON:
                intake.setTargetPower(intakePower);
                if (numberOfTimesIntakeBeamBreakTriggered >= 2) {
                    off();
                }
                break;
            case OFF:
                intake.setTargetPower(0.0);
            case REVERSED:
                intake.setTargetPower(-intakePower);
                if (numberOfTimesIntakeBeamBreakTriggered <= 2) {
                    off();
                }
        }
    }

    public void on() {
        numberOfTimesIntakeBeamBreakTriggered = 0;
        state = State.ON;
    }

    public void off() {
        numberOfTimesIntakeBeamBreakTriggered = 0;
        state = State.OFF;
    }

    public void reverse() {
        state = State.REVERSED;
    }
}
