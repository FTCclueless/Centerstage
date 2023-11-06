package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Hang {
    private final PriorityMotor hang;
    private State state = State.OFF;

    enum State {
        ON,
        REVERSE,
        OFF
    }

    public Hang(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        hang = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "hang"), "hang", 1, 2);

        hardwareQueue.addDevice(hang);
    }

    public void update() {
        switch (state) {
            case ON:
                hang.setTargetPower(1.0);
                break;
            case REVERSE:
                hang.setTargetPower(-1.0);
                break;
            case OFF:
                hang.setTargetPower(0.0);
                break;
        }
    }

    public void on() {
        state = State.ON;
    }

    public void reverse() {
        state = State.REVERSE;
    }

    public void off() {
        state = State.OFF;
    }
}
