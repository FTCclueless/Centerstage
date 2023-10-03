package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Deposit {

    enum AutoState {
        DOWN,
        EXTEND,
        DEPOSIT
    }

    HardwareQueue hardwareQueue;
    Sensors sensors;

    Pose2d target;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        //finish init other classes
    }

    public void setTarget(Pose2d target) {
    }
}
