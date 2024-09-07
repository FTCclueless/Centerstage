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

    public EndAffector(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        Servo[] v4bar = new Servo[] {hardwareMap.get(Servo.class, "V4BarServo1"), hardwareMap.get(Servo.class, "V4BarServo2")};
        v4Servo = new PriorityServo(
            v4bar,
            "V4BarServo",
            PriorityServo.ServoType.AXON_MINI,
            1,
            0,
            1,
            0.9299999,
            false,
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
            0.27498999,
            false,
            1,2,
            new double[] {-1}
        );
        this.sensors = sensors;
//        v4Servo.setCurrentAngle(Deposit.v4BarTransferAngle);

        //hardwareQueue.addDevice(v4Servo);
        //hardwareQueue.addDevice(topServo);
    }

    public boolean checkReady() {
        return v4Servo.inPosition() && topServo.inPosition();
    }

}
