package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Hang {
    private final PriorityServo hangServo1;
    private final PriorityServo hangServo2;

    // TODO: Figure out right angle
    private double holdAngle = 0.0;
    private double releaseAngle = 90.0;

    private State state = State.HOLD;

    enum State {
        HOLD,
        RELEASE
    }

    public Hang(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        hangServo1 = new PriorityServo(hardwareMap.get(Servo.class, "hangServo1"), "hangServo1", PriorityServo.ServoType.AXON_MINI, 1.0, 0.0,1.0,0.0, false, 1.0,1.0);
        hangServo2 = new PriorityServo(hardwareMap.get(Servo.class, "hangServo2"), "hangServo2", PriorityServo.ServoType.AXON_MINI, 1.0, 0.0,1.0,0.0, true, 1.0,1.0);

        hardwareQueue.addDevice(hangServo1);
        hardwareQueue.addDevice(hangServo2);
    }

    public void update() {
        switch (state) {
            case HOLD:
                hangServo1.setTargetAngle(Math.toRadians(holdAngle));
                hangServo2.setTargetAngle(Math.toRadians(holdAngle));
                break;
            case RELEASE:
                hangServo1.setTargetAngle(Math.toRadians(releaseAngle));
                hangServo2.setTargetAngle(Math.toRadians(releaseAngle));
                break;
        }
    }

    public void release() {
        state = State.RELEASE;
    }

    public void hold() {
        state = State.HOLD;
    }
}
