package org.firstinspires.ftc.teamcode.subsystems.hangActuation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class HangActuation {
    public PriorityServo hangActuation;
    public static double downAng = 0.0;
    public static double upAng = 4.43;

    public HangActuation(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        hangActuation = new PriorityServo(hardwareMap.get(Servo.class, "hangActuation"), "hangActuation",
                PriorityServo.ServoType.SPEED,
                1,
                0,
                1,
                0,
                false,
                1, 2);
        hardwareQueue.addDevice(hangActuation);
    }

    public void down() {
        hangActuation.setTargetAngle(downAng, 1);
    }

    public void up() {
        hangActuation.setTargetAngle(upAng, 1);
    }
}
