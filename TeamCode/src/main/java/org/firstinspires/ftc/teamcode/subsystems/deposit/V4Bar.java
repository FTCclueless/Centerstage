package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class V4Bar {
    private final PriorityServo servo;
    private final Sensors sensors;

    public V4Bar(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        // TODO: Value yoink
        servo = new PriorityServo(
            hardwareMap.get(Servo.class, "V4BarServo"),
            "V4BarServo",
            PriorityServo.ServoType.AMAZON,
            1,
            0,
            1,
            0,
            false,
            1, 2
        );
        this.sensors = sensors;
        hardwareQueue.addDevice(servo);
    }
}
