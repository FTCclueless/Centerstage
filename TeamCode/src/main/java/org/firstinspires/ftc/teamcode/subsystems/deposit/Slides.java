package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides {
    private final PriorityMotor slidesMotors;
    public double length;
    public double vel;
    private final Sensors sensors;
    public static double ticksToInches = 0.04132142857142857;
    public static double maxSlidesHeight = 27.891;
    private double targetLength = 0;
    public static double maxVel = 1.6528571428571428;
    public static double kP = 0.15; // used to be 0.11
    public static double kA = 3;
    public static double kStatic = 0.15;
    public static double minPower = 0.22;
    public static double minPowerThresh = 0.8;
    public static double downPower = -0.1;
    public static double forceDownPower = -0.5;

    public Slides(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;

        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "slidesMotor0");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "slidesMotor1");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Globals.RUNMODE != RunMode.TELEOP) {
            Log.e("RESETTTING", "RESTETING SLIDES");
            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        slidesMotors = new PriorityMotor(new DcMotorEx[] {m1, m2}, "slidesMotor", 2, 5, new double[] {-1, -1}, sensors);
        hardwareQueue.addDevice(slidesMotors);
    }

    /**
     * Pretty misleading function name -- Eric
     * @return power
     */
    private double feedforward() {
        double error = targetLength - length;

        TelemetryUtil.packet.put("slidesError", error);

        if (targetLength <= 0.6) {
            error = -4;
        }
        if (length <= Globals.slidesV4Thresh+2 && targetLength <= 0.6)
            return (length <= 0.25? 0 : forceDownPower) + downPower * (12/sensors.getVoltage());
        return (error * (maxVel / kA)) * kP + kStatic + ((Math.abs(error) > minPowerThresh) ? minPower * Math.signum(error) : 0);
    }

    public void update() {
        length = (double) sensors.getSlidesPos() * ticksToInches;
        vel = sensors.getSlidesVelocity() * ticksToInches;
        if (!(Globals.RUNMODE == RunMode.TESTER)) {
            slidesMotors.setTargetPower(Math.max(Math.min(feedforward(), 1), -1));
        }
    }

    public void setTargetLength(double length) {
        targetLength = Math.max(Math.min(length, maxSlidesHeight),0);
    }

    public boolean inPosition(double threshold) {
        return Math.abs(targetLength - length) <= threshold;
    }

    public double getLength() {
        return length;
    }
}
