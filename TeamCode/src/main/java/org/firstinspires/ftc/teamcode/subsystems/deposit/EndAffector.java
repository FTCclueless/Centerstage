package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoMINIP;

public class EndAffector {
    public final PriorityServoMINIP v4Servo;
    public final PriorityServoMINIP botTurret;
    public final PriorityServo topTurret;
    public final Sensors sensors;
    public static double maxYaw = Math.toRadians(45); //todo
    private double bottomAngle;
    private double targetPitch;
    private double topAngle;
    private double power = 1;

    public EndAffector(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        // TODO: Value yoink
        v4Servo = new PriorityServoMINIP(
            hardwareMap.get(Servo.class, "V4BarServo"),
            "V4BarServo",
            1,
            0,
            1,
            0,
            false,
            1, 2,
            hardwareMap.get(AnalogInput.class, "V4BarServoEncoder")
        );
        botTurret = new PriorityServoMINIP(
            hardwareMap.get(Servo.class, "bottomTurret"),
            "bottomTurret",
            1,
            0,
            1,
            0,
            false,
            1, 2,
            hardwareMap.get(AnalogInput.class, "bottomTurretEncoder")
        );
        topTurret = new PriorityServoMINIP(
            hardwareMap.get(Servo.class, "topTurret"),
            "topTurret",
            1,
            0,
            1,
            0,
            false,
            1,2,
                hardwareMap.get(AnalogInput.class, "topTurretEncoder")
        );
        this.sensors = sensors;
        //hardwareQueue.addDevice(v4Servo);
        //hardwareQueue.addDevice(botTurret);
        //hardwareQueue.addDevice(topTurret);
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
