package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    enum State {
        READY,
        BUSY
    };

    private final PriorityMotor slidesMotors;
    public State state = State.READY;
    public double length;
    private final Sensors sensors;
    public static double angle = Math.toRadians(30);
    public static double ticksToRadians = 0; // TODO
    public static double radiansToInches = 0; // TODO
    public static double maxSlidesHeight = 0; // TODO
    public static double slidesThreshold = 0.5;
    private double targetLength = 0;
    public static double maxVel = 0; // TODO
    public static double kP = 0; // TODO
    public static double kA = 0; // TODO
    public static double kStatic = 0; // TODO

    public Slides(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;

        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesMotors = new PriorityMotor(new DcMotorEx[] {m1, m2}, "slidesMotor", 2, 5);
        hardwareQueue.addDevice(slidesMotors);
    }

    public double feedforward() {
        double error = targetLength - length;
        return (error * (maxVel / kA)) * kP + kStatic; // todo fix
    }

    public void update() {
        length = sensors.getSlides() * ticksToRadians * radiansToInches;

        switch (state) {
            case READY:
                break;
            case BUSY:
                slidesMotors.setTargetPower(feedforward());

                // 2nd check for redundancy
                if (Math.abs(targetLength - length) <= slidesThreshold || (sensors.isSlidesDown() && targetLength <= slidesThreshold))
                    state = State.READY;
                break;
        }
    }

    public void setLength(double length) {
        targetLength = Math.min(length, maxSlidesHeight);
        state = State.BUSY;
    }
}
