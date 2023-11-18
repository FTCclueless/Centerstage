package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoMINIP;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoTunable;

public class EndAffector {
    public final PriorityServo v4Servo;
    public final PriorityServo botTurret;
    public final PriorityServo topTurret;
    public final Sensors sensors;
    public static double maxYaw = Math.toRadians(45); //todo
    private double bottomAngle;
    private double targetPitch;
    private double topAngle;
    private double power = 0.5;

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
            0,
            false,
            1, 2,
            new double[] {1, -1}
        );
        botTurret = new PriorityServo(
            hardwareMap.get(Servo.class, "bottomTurret"),
            "bottomTurret",
            PriorityServo.ServoType.PRO_MODELER,
            1,
            0,
            1,
            0.3969999999999999,
            false,
            1, 2
        );
        topTurret = new PriorityServo(
            hardwareMap.get(Servo.class, "topTurret"),
            "topTurret",
            PriorityServo.ServoType.PRO_MODELER,
            1,
            0,
            1,
            0,
            false,
            1, 2
        );
        this.sensors = sensors;
        hardwareQueue.addDevice(v4Servo);
        hardwareQueue.addDevice(botTurret);
        hardwareQueue.addDevice(topTurret);
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setBotTurret(double ang) {
        bottomAngle = ang;
        botTurret.setTargetAngle(Math.max(Math.min(ang,maxYaw),-maxYaw), power);
    }
    public void setTopTurret(double ang) {
        topAngle = ang;
        topTurret.setTargetAngle(ang, power);
    }
    public void setV4Bar(double pitch) {
        targetPitch = pitch;
        v4Servo.setTargetAngle(pitch, power);
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
