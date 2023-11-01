package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class EndAffector {
    public final PriorityServo v4Servo;
    public final PriorityServo botTurret;
    public final PriorityServo topTurret;
    public final Sensors sensors;
    public static double maxYaw = Math.toRadians(45); //todo
    private double bottomAngle;
    private double targetPitch;
    private double topAngle;

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
        hardwareQueue.addDevice(botTurret);
        hardwareQueue.addDevice(topTurret);
    }

    public void setBotTurret(double ang) {
        bottomAngle = ang;
        botTurret.setTargetAngle(Math.max(Math.min(ang,maxYaw),-maxYaw), 1.0);
    }
    public void setTopTurret(double ang) {
        topAngle = ang;
        topTurret.setTargetAngle(ang, 1.0);
    }
    public void setV4Bar(double pitch) {
        targetPitch = pitch;
        v4Servo.setTargetAngle(pitch, 1.0);
    }

    public double getBottomAngle() {
        return bottomAngle;
    }
    public double getTopAngle() {
        return topAngle;
    }
    public double getTargetPitch() {
        return targetPitch;
    }

    public boolean checkBottom() {
        return (botTurret.getCurrentAngle() == bottomAngle);
    }
    public boolean checkV4() {
        return (v4Servo.getCurrentAngle() == targetPitch);
    }
    public boolean checkTop() {
        return (topTurret.getCurrentAngle() == topAngle);
    }
    public boolean checkReady() {
        return checkBottom() && checkTop() && checkV4();
    }

}
