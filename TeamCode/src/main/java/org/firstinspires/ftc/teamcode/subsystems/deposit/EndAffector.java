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
    public final PriorityServo topServo;
    public final Sensors sensors;
    public static double maxYaw = Math.toRadians(45); //todo
    private double bottomAngle;
    private double targetPitch;
    private double topAngle;
    private double topPitch;
    private double power = 1;

    public EndAffector(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        Servo[] v4bar = new Servo[] {hardwareMap.get(Servo.class, "V4BarServo1"), hardwareMap.get(Servo.class, "V4BarServo2")};
        // TODO: Value yoink
        v4Servo = new PriorityServo(
            v4bar,
            "V4BarServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.25, //0.2609
            //0, Math.toRadians(240),
            false,
            1, 2,
            new double[] {1, -1}
        );
        botTurret = new PriorityServo(
            hardwareMap.get(Servo.class, "bottomTurret"),
            "bottomTurret",
            PriorityServo.ServoType.AXON_MINI_SCUFF_TURRET,
            1,
            0,
            1,
            0.259,
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
            0.5,
            false,
            1, 2
        );
        topServo = new PriorityServo(
            hardwareMap.get(Servo.class, "topServo"),
            "topServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.492,
            false,
            1,2
        );
        this.sensors = sensors;
        hardwareQueue.addDevice(v4Servo);
        hardwareQueue.addDevice(botTurret);
        hardwareQueue.addDevice(topTurret);
        hardwareQueue.addDevice(topServo);
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setBotTurret(double ang) {
        bottomAngle = ang;
        botTurret.setTargetAngle(ang, power);
    }
    public void setTopTurret(double ang) {
        topAngle = ang;
        topTurret.setTargetAngle(ang, power);
    }
    public void setV4Bar(double pitch) {
        targetPitch = pitch;
        v4Servo.setTargetAngle(pitch, power);
    }
    public void setTopServo(double pitch) {
        topPitch = pitch;
        topServo.setTargetAngle(pitch, power);
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
    public double getTopPitch() {
        return topPitch;
    }

    public boolean checkBottom() {
        return (botTurret.getCurrentAngle() == bottomAngle);
    }
    public boolean checkV4() {
        return (v4Servo.getCurrentAngle() == targetPitch);
    }
    public boolean checkTopTurret() {
        return (topTurret.getCurrentAngle() == topAngle);
    }
    public boolean checkTopServo() { return (topServo.getCurrentAngle() == topAngle); }
    public boolean checkReady() {
        return checkBottom() && checkTopTurret() && checkV4() && checkTopServo();
    }

}
