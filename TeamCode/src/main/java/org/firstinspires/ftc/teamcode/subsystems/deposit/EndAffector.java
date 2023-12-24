package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServoAxonEnc;

@Config
public class EndAffector {
    public final PriorityServoAxonEnc v4Servo;
    public final PriorityServo topServo;
    public final Sensors sensors;

    public static double v4MinDist = 45;
    public static double v4SlowDown = 0.2; // 0.667

    public EndAffector(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        Servo[] v4bar = new Servo[] {hardwareMap.get(Servo.class, "V4BarServo1"), hardwareMap.get(Servo.class, "V4BarServo2")};
        v4Servo = new PriorityServoAxonEnc(
            v4bar,
            sensors,
            0,
            "V4BarServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.787, //0.2609
            0.855,
            false,
            true,
            Math.toRadians(v4MinDist), v4SlowDown,
            1, 2,
            new double[] {-1, 1}
        );
        topServo = new PriorityServo(
            new Servo[] {hardwareMap.get(Servo.class, "topServo")},
            "topServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.5,
            false,
            1,2,
            new double[] {-1}
        );
        this.sensors = sensors;
        v4Servo.setCurrentAngle(Deposit.v4BarIntakeAngle);

        hardwareQueue.addDevice(v4Servo);
        hardwareQueue.addDevice(topServo);
    }

    public boolean checkReady() {
        return v4Servo.inPosition() && topServo.inPosition();
    }

}
