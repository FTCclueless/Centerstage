package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Intake {
    enum State {
        ON,
        OFF,
        REVERSED
    }

    private final PriorityMotor intake;
    public State state = State.ON;
    private final Sensors sensors;
    private final double intakePower = 0.5;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2);

        hardwareQueue.addDevice(intake);
    }

    public void update() {
        switch (state) {
            case ON:
                intake.setTargetPower(intakePower);
                break;
            case OFF:
                intake.setTargetPower(0.0);
            case REVERSED:
                intake.setTargetPower(-intakePower);
        }
    }

    public void on() {
        state = State.ON;
    }

    public void off() {
        state = State.OFF;
    }

    public void reversed() {
        state = State.REVERSED;
    }
}
