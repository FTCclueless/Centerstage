package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
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

    public static double slidesV4Thresh = 3;

    // v4bar angles
    public static double v4BarIntakeAngle = -1.10388;
    public static double v4BarTransferAngle = -1.2088;
    public static double v4BarDepositAngle = 1.38;

    // top servo angles (if the top servo is even in the design)
    public static double topServoIntakeAngle = 1.115;
    public static double topServoDepositAngle = 0.0;

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
        this.targetH = targetH;
        this.targetX = targetX;

        if (state == State.INTAKE) {
            state = State.START_DEPOSIT;
        }
    }

    public void releaseOne() {
        release.releaseOne();
    }

    public void releaseTwo() {
        release.releaseTwo();
    }

    public void retract() {
        state = State.RETRACT;
    }

    public boolean checkReady() {
        return endAffector.checkReady() && !slides.inPosition(2);
    }

    public void update() {
        TelemetryUtil.packet.put("Deposit State", state);

        switch (state) {
            case START_DEPOSIT:
                release.hold();

                if (release.inPosition()) {
                    slides.setTargetLength(Math.max(targetH, slidesV4Thresh + 2));
                }

                if (slides.getLength() > slidesV4Thresh) {
                    state = State.FINISH_DEPOSIT;
                }
                break;
            case FINISH_DEPOSIT: // stuck in this state unless someone calls dunk method. In the meantime it will constantly update targetH
                endAffector.v4Servo.setTargetAngle(v4BarDepositAngle,1.0);
                endAffector.topServo.setTargetAngle(topServoDepositAngle,1.0);

                if (endAffector.v4Servo.getCurrentAngle() > Math.toRadians(90)) { // TODO: Tune this value
                    slides.setTargetLength(targetH);
                }

                if (endAffector.v4Servo.inPosition() && slides.inPosition(2)) {
                    state = State.DEPOSIT;
                }
                break;
            case DEPOSIT:
                if (release.readyToRetract()) {
                    state = State.START_RETRACT;
                }
                break;
            case START_RETRACT:
                release.intake();

                endAffector.v4Servo.setTargetAngle(v4BarIntakeAngle,1.0);
                endAffector.topServo.setTargetAngle(topServoIntakeAngle, 1.0);

                slides.setTargetLength(slidesV4Thresh + 2);

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

                if (robot.intake.state == Intake.State.ON && sensors.getSlidesPos() < 3) {
                    endAffector.v4Servo.setTargetAngle(v4BarTransferAngle,1.0);
                } else {
                    endAffector.v4Servo.setTargetAngle(v4BarIntakeAngle,1.0);
                }

                endAffector.topServo.setTargetAngle(topServoIntakeAngle,1.0);
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
