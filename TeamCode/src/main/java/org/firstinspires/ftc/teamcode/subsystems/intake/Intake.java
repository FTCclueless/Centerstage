package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Intake {
    enum State {
        ON,
        OFF,
        REVERSED
    }

    private final PriorityMotor intake;
    public PriorityServo actuation;
    private State state = State.OFF;
    private final Sensors sensors;

    public static double intakePower = 0.5; // TODO: Made this editable in FTC dashboard
    private double actuationHeight = 1.0;

    private boolean alreadyTriggered = false;
    private int numberOfTimesIntakeBeamBreakTriggered = 0;

    double actuationLength = 3.5;
    double actuationAngle = 0.0;

    private double delayToTurnOffIntake = 50; // ms
    private long startTime = 0; // ms
    private boolean isAlreadyTriggered = false;

    public boolean isReady = false;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.sensors = sensors;
        intake = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intake", 1, 2, -1);
        actuation = new PriorityServo(
                hardwareMap.get(Servo.class,"actuation"),
                "actuation",
                PriorityServo.ServoType.PRO_MODELER,
                1.0,
                0.0,
                0.915,
                0.663,
                false,
                1.0,
                1.0
        );

        this.state = State.OFF;
        hardwareQueue.addDevice(intake);
        hardwareQueue.addDevice(actuation);
    }

    double maxHeightAtParallel = 2.4;

    public void update() {
        actuation.setTargetAngle(actuationAngle, 1.0);

        /*if (sensors.isIntakeTriggered() && !alreadyTriggered) {
            alreadyTriggered = true;
            if (state == State.ON) {
                numberOfTimesIntakeBeamBreakTriggered++;
            } else {
                numberOfTimesIntakeBeamBreakTriggered--;
            }
        }
        if (!sensors.isIntakeTriggered()) {
            alreadyTriggered = false;
        }

        if (numberOfTimesIntakeBeamBreakTriggered > 2) {
            reverse();
        }*/

        // TODO: Might need to have a delay bc pixels may not have reached transfer - Huddy kim apparently
        switch (state) {
            case ON:
                intake.setTargetPower(intakePower);
                /*if (numberOfTimesIntakeBeamBreakTriggered >= 2) {
                    if (!isAlreadyTriggered) {
                        isAlreadyTriggered = true;
                        startTime = System.currentTimeMillis();
                    }
                    if (System.currentTimeMillis() - startTime >= delayToTurnOffIntake) {
                        off();
                        isReady = true;
                    }
                }*/
                break;
            case OFF:
                intake.setTargetPower(0.0);
                break;
            case REVERSED:
                intake.setTargetPower(-0.8);
//                if (numberOfTimesIntakeBeamBreakTriggered <= 2) {
//                    off();
//                }
                break;
        }
    }

    public void on() {
        numberOfTimesIntakeBeamBreakTriggered = 0;
        isReady = false;
        state = State.ON;
    }

    public void off() {
        isReady = true;
        isAlreadyTriggered = false;
        numberOfTimesIntakeBeamBreakTriggered = 0;
        state = State.OFF;
    }

    public void toggle() {
        if (state == State.ON) {
            off();
        } else if (state == State.OFF) {
            on();
        }
    }

    public void reverse() {
        state = State.REVERSED;
    }

    public void setActuationAngle (double angle) {
        actuationAngle = angle;
    }

    public void actuationDown () {
        actuationAngle = 30;
    }

    public void actuationUp () {
        actuationAngle = -90;
    }

    public void actuationSinglePixel () {
        actuationAngle = 28;
    }

    public double getIntakeActuationOffset() {
        return Math.cos(actuation.getCurrentAngle()) * actuationLength;
    }
}