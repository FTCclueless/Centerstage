package org.firstinspires.ftc.teamcode.subsystems.airplane;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Airplane {
    private final PriorityServo airplane;

    private double holdAngle = 0.9469;
    private double releaseAngle = 1.8155;

    private State state = State.HOLD;

    enum State {
        HOLD,
        RELEASE
    }

    public Airplane(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        airplane = new PriorityServo(hardwareMap.get(Servo.class, "airplane"), "airplane", PriorityServo.ServoType.AXON_MINI, 1.0, 0.0,1.0,0.0, false, 1.0,1.0);

        hardwareQueue.addDevice(airplane);
    }

    public void update() {
        switch (state) {
            case HOLD:
                airplane.setTargetAngle(holdAngle, 1.0);
                break;
            case RELEASE:
                airplane.setTargetAngle(releaseAngle, 1.0);
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
