package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class EndAffector {
    private final PriorityServo v4Servo;
    private final Sensors sensors;
    private final double intakeYaw = Math.PI;
    private final double intakePitch = Math.toRadians(135); //todo
    public double targetYaw;
    public double targetPitch;

    public EndAffector(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        // TODO: Value yoink
        v4Servo = new PriorityServo(
            hardwareMap.get(Servo.class, "V4BarServo"),
            "V4BarServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0,
            false,
            1, 2
        );
        this.sensors = sensors;
        hardwareQueue.addDevice(v4Servo);
    }

    public void setYaw(double yaw) {
        targetYaw = yaw;
    }
    public void setPitch(double pitch) {
        targetPitch = pitch;
    }

}
