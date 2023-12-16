package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class EndAffector {
    public final PriorityServo v4Servo;
    public final PriorityServo botTurret;
    public final PriorityServo topTurret;
    public final PriorityServo topServo;
    public final Sensors sensors;
    public static double maxYaw = Math.toRadians(45); //todo

    public static double botTurretMinDist = 45; //temp
    public static double botTurretSlowPow = 0.4;
    public static double v4MinDist = 45;
    public static double v4SlowDown = 0.667;

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
            0.753, //0.2609
            false,
            Math.toRadians(0), 0,
            1, 2,
            new double[] {-1, 1}
        );
        botTurret = new PriorityServo(
            hardwareMap.get(Servo.class, "bottomTurret"),
            "bottomTurret",
            PriorityServo.ServoType.AXON_MAX,
            1,
            0,
            1,
            0.333,
            false,
            Math.toRadians(0), 0,
            1, 2,
            new double[] {-1} //basevolt = 0.189 --Kyle
        );
        topTurret = new PriorityServo(
            hardwareMap.get(Servo.class, "topTurret"),
            "topTurret",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.113,
            false,
            1, 2
        );
        topServo = new PriorityServo(
            new Servo[] {hardwareMap.get(Servo.class, "topServo")},
            "topServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.519,
            false,
            1,2,
            new double[] {-1}
        );
        this.sensors = sensors;
        hardwareQueue.addDevice(v4Servo);
        hardwareQueue.addDevice(botTurret);
        hardwareQueue.addDevice(topTurret);
        hardwareQueue.addDevice(topServo);

        v4Servo.setCurrentAngle(Deposit.downPitch);
        botTurret.setCurrentAngle(Deposit.intakeBotTurret);
    }

    public boolean checkReady() {
        return v4Servo.inPosition() && topTurret.inPosition() && botTurret.inPosition() && topServo.inPosition();
    }

}
