package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    private final PriorityMotor slidesMotors;
    private double length;
    public double vel;
    private final Sensors sensors;
    public static double ticksToInches = 0.04132142857142857;
    public static double maxSlidesHeight = 28.3465;
    private double targetLength = 0;
    public static double maxVel = 1.6528571428571428;
    public static double kP = 0.13;
    public static double kA = 3;
    public static double kStatic = 0.15;
    public static double threshold = 2;
    public static double minPower = 0.25;
    public static double minPowerThresh = 0.8;
    public static double downPower = -0.2; // JANK

    public Slides(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;

        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesMotors = new PriorityMotor(new DcMotorEx[] {m1, m2}, "slidesMotor", 2, 5, new double[] {-1, -1});
        hardwareQueue.addDevice(slidesMotors);
    }

    /**
     * Pretty misleading function name -- Eric
     * @return power
     */
    private double feedforward() {
        double error = targetLength - length;
        TelemetryUtil.packet.put("Error", error);
        if (length <= 3 && targetLength <= 0.6) {
            return downPower;
        } else {
            return (error * (maxVel / kA)) * kP + kStatic + ((Math.abs(error) > minPowerThresh) ? minPower * Math.signum(error) : 0);
        }
    }

    public void update() {
        length = sensors.getSlidesPos() * ticksToInches;
        vel = sensors.getSlidesVelocity() * ticksToInches;
        slidesMotors.setTargetPower(Math.max(Math.min(feedforward(), 1),-1));
    }

    public void setLength(double length) {
        targetLength = Math.min(length, maxSlidesHeight);
        Log.e("slides target length", length + "");
    }

    public boolean isBusy() {
        return Math.abs(targetLength - length) <= threshold;
    }

    public double getLength() {
        return length;
    }
}
