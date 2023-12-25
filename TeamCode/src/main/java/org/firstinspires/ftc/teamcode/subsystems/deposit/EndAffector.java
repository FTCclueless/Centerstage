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
    public final PriorityServo topServo;
    public final Sensors sensors;

    public static double v4MinSlowDownDist = Math.toRadians(45);
    public static double v4SlowDownPower = 0.25;

    public EndAffector(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        Servo[] v4bar = new Servo[] {hardwareMap.get(Servo.class, "V4BarServo1"), hardwareMap.get(Servo.class, "V4BarServo2")};
        v4Servo = new PriorityServo(
            v4bar,
            "V4BarServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.59699, //0.2609
            false,
            v4MinSlowDownDist,
            v4SlowDownPower,
            1,
            2,
            new double[] {-1, 1}
        );
        topServo = new PriorityServo(
            new Servo[] {hardwareMap.get(Servo.class, "topServo")},
            "topServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.34699,
            false,
            1,2,
            new double[] {-1}
        );
        this.sensors = sensors;
        v4Servo.setCurrentAngle(Deposit.v4BarTransferAngle);

        hardwareQueue.addDevice(v4Servo);
        hardwareQueue.addDevice(topServo);
    }

    public boolean checkReady() {
        return v4Servo.inPosition() && topServo.inPosition();
    }

}
