package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

@Config
public class Deposit {
    public enum State {
        START_DEPOSIT,
        FINISH_DEPOSIT,
        DEPOSIT,
        START_RETRACT,
        RETRACT,
        INTAKE,
        IDLE,
    };
    public State state;

    public Slides slides;
    public EndAffector endAffector;
    public Release release;
    HardwareQueue hardwareQueue;
    Robot robot;
    Sensors sensors;

    double targetH = 0.0;
    double targetX = 0.0;

    public static double slidesV4Thresh = 5;

    // v4bar angles
    public static double v4BarTransferAngle = -0.29690;
    public static double v4BarGrabAngle = -0.2913000;
    public static double v4BarDepositAngle = -2.97538;

    // top servo angles
    public static double topServoTransferAngle = -0.834946;
    public static double topServoGrabAngle = -0.86290085;
    public static double topServoDepositAngle = 2.101297;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.robot = robot;

        state = State.INTAKE;

        slides = new Slides(hardwareMap, hardwareQueue, sensors);
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
        release = new Release(hardwareMap, hardwareQueue);
    }

    // Call this whenever you want! It can be an updating function!
    public void depositAt(double targetH, double targetX) {
        this.targetH = Math.max(Math.min(targetH, Slides.maxSlidesHeight),0);
        this.targetX = targetX;

        if (state == State.INTAKE) {
            state = State.START_DEPOSIT;
            beginDepositTime = System.currentTimeMillis();
        }
    }

    public void releaseOne() {
        release.releaseOne();
    }

    public void releaseTwo() {
        release.releaseTwo();
    }

    public void retract() {
        state = State.START_RETRACT;
    }

    public boolean checkReady() {
        return endAffector.checkReady() && !slides.inPosition(2);
    }

    long beginDepositTime;

    public void update() {
        TelemetryUtil.packet.put("Deposit State", state);

        switch (state) {
            case START_DEPOSIT:
                endAffector.v4Servo.setTargetAngle(v4BarGrabAngle, 0.5);
                endAffector.topServo.setTargetAngle(topServoGrabAngle, 1.0);

                Log.e("slides length", slides.getLength() + "");

                if (endAffector.v4Servo.inPosition() && endAffector.topServo.inPosition()) {
                    release.close();

                    Log.e("System.currentTimeMillis() - beginDepositTime", System.currentTimeMillis() - beginDepositTime + "");
                    if (System.currentTimeMillis() - beginDepositTime > 125) {
                        slides.setTargetLength(Math.max(targetH, slidesV4Thresh + 2));
                    }
                } else {
                    beginDepositTime = System.currentTimeMillis();
                    slides.setTargetLength(0.0);
                }

                if (slides.getLength() > slidesV4Thresh) {
                    state = State.FINISH_DEPOSIT;
                }
                break;
            case FINISH_DEPOSIT: // stuck in this state unless someone calls dunk method. In the meantime it will constantly update targetH
                slides.setTargetLength(targetH);
                endAffector.v4Servo.setTargetAngle(v4BarDepositAngle,0.5);
                endAffector.topServo.setTargetAngle(topServoDepositAngle,1.0);

                if (endAffector.v4Servo.getCurrentAngle() > Math.toRadians(90)) { // TODO: Tune this value
                    slides.setTargetLength(targetH);
                }

                if (endAffector.v4Servo.inPosition() && slides.inPosition(2)) {
                    state = State.DEPOSIT;
                }
                break;
            case DEPOSIT:
                slides.setTargetLength(targetH);

                if (release.readyToRetract()) {
                    state = State.START_RETRACT;
                }
                break;
            case START_RETRACT:
                release.close();
                endAffector.v4Servo.setTargetAngle(v4BarTransferAngle, 0.5);

                if (endAffector.v4Servo.getCurrentAngle() <= Math.toRadians(135)) {
                    slides.setTargetLength(slidesV4Thresh + 2);
                    if(endAffector.v4Servo.getCurrentAngle() <= Math.toRadians(90)) {
                        endAffector.topServo.setTargetAngle(topServoTransferAngle, 1.0);
                    }
                    else{
                        endAffector.topServo.setTargetAngle(0,1.0);
                    }
                }

                if (endAffector.v4Servo.inPosition()) {
                    state = State.RETRACT;
                }
                break;
            case RETRACT:
                slides.setTargetLength(0.0);

                if (slides.inPosition(2)) {
                    state = State.INTAKE;
                }
                break;
            case INTAKE:
                release.intake();
                slides.setTargetLength(0.0);

                endAffector.v4Servo.setTargetAngle(v4BarTransferAngle,0.5);
                endAffector.topServo.setTargetAngle(topServoTransferAngle,1.0);
                break;
            case IDLE: // We are boring :(
                break;
        }
        slides.update();
        release.update();

        TelemetryUtil.packet.put("v4ServoAngle", endAffector.v4Servo.getCurrentAngle());
        TelemetryUtil.packet.put("v4ServoTarget", endAffector.v4Servo.getTargetPosition());
    }
}
