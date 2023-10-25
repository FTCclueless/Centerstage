package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class EndAffector {
    private final PriorityServo v4Servo;
    private final PriorityServo botTurret;
    private final PriorityServo topTurret;
    private final Sensors sensors;
    private final double intakeYaw = Math.PI;
    private final double intakePitch = Math.toRadians(135); //todo
    public double bottomAngle;
    public double targetPitch;
    public double topAngle;

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
        botTurret = new PriorityServo(
                hardwareMap.get(Servo.class, "bottomTurret"),
                "bottomTurret",
                PriorityServo.ServoType.AXON_MINI,
                1,
                0,
                1,
                0,
                false,
                1, 2
        );
        topTurret = new PriorityServo(
                hardwareMap.get(Servo.class, "topTurret"),
                        "topTurret",
                        PriorityServo.ServoType.AXON_MINI,
                        1,
                        0,
                        1,
                        0,
                        false,
                        1,2
        );
        this.sensors = sensors;
        hardwareQueue.addDevice(v4Servo);
    }

    public void setBotTurret(double ang) {
        bottomAngle = ang;
        botTurret.setTargetAngle(ang, 0.75);
    }
    public void setTopTurret(double ang) {
        topAngle = ang;
        topTurret.setTargetAngle(ang, 0.75);
    }
    public void setV4Bar(double pitch) {
        targetPitch = pitch;
        v4Servo.setTargetAngle(pitch, 0.75);
    }

}
