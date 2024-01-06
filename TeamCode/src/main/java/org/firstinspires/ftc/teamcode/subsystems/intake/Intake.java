package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    public enum State {
        ON,
        OFF,
        REVERSED,
        SOFT_REVERSED,
        REVERSE_FOR_TIME
    }

    public final PriorityMotor intake;
    public PriorityServo actuation;
    public State state = State.OFF;
    private final Sensors sensors;
    private final Robot robot;
    public static double intakeCurrent;

    public static double intakePower = 1.0; // TODO: Made this editable in FTC dashboard

    double actuationLength = 3.5;
    double[] actuationAngles = new double[] {0.6984, 0.5087, 0.3376, 0.21738, 0.08325};
    double actuationFullyUpAngle = -0.314508;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.sensors = sensors;
        this.robot = robot;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2, -1, sensors);
        actuation = new PriorityServo(
                hardwareMap.get(Servo.class,"actuation"),
                "actuation",
                PriorityServo.ServoType.TORQUE,
                1.0,
                0.0,
                1.0,
                0.068,
                false,
                1.0,
                1.0
        );

        this.state = State.OFF;
        hardwareQueue.addDevice(intake);
        hardwareQueue.addDevice(actuation);
    }

    public void update() {
        TelemetryUtil.packet.put("Intake State", state);

        switch (state) {
            case ON:
                intake.setTargetPower(intakePower);
                intakeCurrent = intake.motor[0].getCurrent(CurrentUnit.AMPS);
                break;
            case OFF:
                intake.setTargetPower(0.0);
                break;
            case REVERSED:
                intake.setTargetPower(-1.0);
                break;
            case SOFT_REVERSED:
                intake.setTargetPower(-0.35);
                break;
            case REVERSE_FOR_TIME:
                if (System.currentTimeMillis() - reverseForSomeTimeStart < time) {
                    intake.setTargetPower(-1.0);
                } else {
                    state = previousState;
                }
                break;
        }
    }

    public void on() {
        state = State.ON;
    }

    public void off() {
        state = State.OFF;
    }

    public void reverse() {
        state = State.REVERSED;
    }

    long reverseForSomeTimeStart;
    double time;
    State previousState;
    public void reverseForSomeTime(double time) {
        this.time = time;
        reverseForSomeTimeStart = System.currentTimeMillis();

        if (state != State.REVERSE_FOR_TIME) {
            previousState = state;
        }
        state = State.REVERSE_FOR_TIME;
    }

    public void softReverse() {
        state = State.SOFT_REVERSED;
    }

    public void actuationFullyDown() {
        actuation.setTargetAngle(actuationAngles[0],1.0);
    }

    public void actuationFullyUp() {
        actuation.setTargetAngle(actuationFullyUpAngle, 1.0);
    }

    public void setActuationHeight (int pixel) {
        actuation.setTargetAngle(actuationAngles[pixel], 1.0);
    }

    public void setActuationAngle(double angle, double power) { // 0 index based
        actuation.setTargetAngle(angle, power);
    }

    public double getIntakeActuationOffset() {
        return Math.cos(actuation.getCurrentAngle()) * actuationLength;
    }

    public boolean isActuationUp() {
        return actuation.getCurrentAngle() == actuationFullyUpAngle;
    }
}