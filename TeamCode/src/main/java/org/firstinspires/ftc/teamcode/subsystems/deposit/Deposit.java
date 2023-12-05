package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

@Config
public class Deposit {
    DepositMath depositMath;
    public enum State {
        START_DEPOSIT,
        MOVE_V4UP,
        FINISH_DEPOSIT,
        EXTEND_ROTATE180,
        WAIT_DUNK,
        START_RETRACT,
        RETRACT_ROTATE180,
        MOVE_V4DOWN,
        DOWN,
        WAIT,
    };
    public State state;

    public Slides slides;
    public EndAffector endAffector;
    public Dunker dunker;
    HardwareQueue hardwareQueue;
    Sensors sensors;

    Pose2d targetBoard;
    double targetY;
    double targetH;

    double xError = 5;
    double yError = 5;
    double headingError = 0;
    double xOffset = 2;
    public static double intakePitch = -1.201197; //todo
    public static double slidesV4Thresh = 12; //todo
    public static double upPitch = 1.5215;
    public static double depositTopTurret = 0.0;
    public static double intakeTopTurret = 0.1047197;
    public static double intakeTopServoAngle = 1.34288;
    public static double intakeBotTurret = -3.03323619;

    boolean inPlace = false;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        depositMath = new DepositMath();

        state = State.DOWN;

        slides = new Slides(hardwareMap, hardwareQueue, sensors);
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
        dunker = new Dunker(hardwareMap, hardwareQueue, sensors);

        //finish init other classes
    }

    public void setTargetBoard(Pose2d targetBoard) {
        this.targetBoard = targetBoard;
    }

    // Call this whenever you want! It can be an updating function!
    public void depositAt(double targetH, double targetY, double xError) {
        this.targetH = targetH;
        this.targetY = targetY;
        this.xError = xError;

        if (state == State.DOWN)
            state = State.START_DEPOSIT;
    }

    public void depositAt(double targetH, double targetY) {
        this.depositAt(targetH, targetY, 4);
    }

    public void resetXOffset() {
        xOffset = 2;
    }

    public void dunk(int numPixels) {
        numPixels = Math.min(Math.max(0, numPixels), 2);
        if (numPixels == 2) {
            dunker.dunk2();
        }
        else {
            dunker.dunk1();
        }
        state = State.WAIT_DUNK;
        /* Deposit this number of pixels and because its one servo we don't need none of that state yucky yucky (I THINK??) */
    }

    public void inPlace() {
        inPlace = true;
    }

    public void unInPlace() {
        inPlace = false;
    }

    public void retract() {
        state = State.START_RETRACT;
    }

    public void update() {
        TelemetryUtil.packet.put("depoState", state);
        switch (state) {
            case START_DEPOSIT: // any adjustments initialize here --Kyle
                targetY = 0; // temporary to remove bottom turret
                if (Globals.RUNMODE == RunMode.TELEOP) {
                    depositMath.calculate(
                        xError,
                        0,
                        0,
                        targetH, targetY
                    );
                } else {
                    if (inPlace) {
                        xError = targetBoard.x - ROBOT_POSITION.x;
                        yError = targetBoard.y - ROBOT_POSITION.y;
                        headingError = targetBoard.heading - ROBOT_POSITION.heading;
                    }

                    depositMath.calculate(
                        xError,
                        yError,
                        headingError,
                        targetH, targetY
                    );
                }
                slides.setLength(Math.max(depositMath.slideExtension, slidesV4Thresh));

                if (slides.getLength() > slidesV4Thresh)
                    state = State.MOVE_V4UP;

                break;
            case MOVE_V4UP:
                endAffector.setV4Bar(upPitch);
                if (endAffector.v4Servo.getCurrentAngle() == upPitch)
                    state = State.EXTEND_ROTATE180;
                break;

            case EXTEND_ROTATE180:
                endAffector.setBotTurret(depositMath.v4BarYaw);
                Log.e("depositMath.v4BarYaw", depositMath.v4BarYaw + "");
                if (endAffector.checkBottom()) {
                    state = State.FINISH_DEPOSIT;
                }
                break;
            case FINISH_DEPOSIT: // Also our update state -- Eric
                if (Globals.RUNMODE == RunMode.TELEOP) {
                    depositMath.calculate(
                            xOffset,
                            0,
                            0,
                            targetH, targetY
                    );
                } else {
                    if (inPlace) {
                        xError = targetBoard.x - ROBOT_POSITION.x;
                        yError = targetBoard.y - ROBOT_POSITION.y;
                        headingError = targetBoard.heading - ROBOT_POSITION.heading;
                    }

                    TelemetryUtil.packet.put("xError", xError);
                    TelemetryUtil.packet.put("yError", yError);
                    TelemetryUtil.packet.put("headingError", headingError);

                    depositMath.calculate(
                            xError,
                            yError,
                            headingError,
                            targetH, targetY
                    );
                }
                TelemetryUtil.packet.put("slideExtension: ", depositMath.slideExtension);
                TelemetryUtil.packet.put("v4Yaw", depositMath.v4BarYaw);

                slides.setLength(depositMath.slideExtension);
                endAffector.setBotTurret(depositMath.v4BarYaw);
                endAffector.setV4Bar(depositMath.v4BarPitch);
                endAffector.setTopServo(-depositMath.v4BarPitch);

                Log.e("topServo angle", -depositMath.v4BarPitch + "");

                if (depositMath.v4BarPitch < 0) {
                    //endAffector.setBotTurret(0);
                    Log.e("v4bar too low", "E");
                }

                if (Globals.RUNMODE == RunMode.TELEOP) {
                    endAffector.setTopTurret(-depositMath.v4BarYaw);
                    Log.e("topTurret angle", -depositMath.v4BarYaw + "");
                } else {
                    endAffector.setTopTurret(targetBoard.heading - ROBOT_POSITION.heading - depositMath.v4BarYaw);
                }

                break;
            case WAIT_DUNK:
                if (Globals.RUNMODE == RunMode.AUTO && dunker.dunkState == Dunker.DunkState.CHILL) {
                    state = State.START_RETRACT;
                }
                break;

            case START_RETRACT:
                //endAffector.setBotTurret(0);
                slides.setLength(slidesV4Thresh);
                endAffector.setV4Bar(upPitch);
                endAffector.setTopServo(intakeTopServoAngle);
                endAffector.setTopTurret(intakeTopTurret);
                /* move v4bar servo to minimum value before bricking */

                if (endAffector.v4Servo.getCurrentAngle() == upPitch)
                    state = State.RETRACT_ROTATE180; //skipping for same reason --kyle

                break;
            case RETRACT_ROTATE180:
                endAffector.setBotTurret(intakeBotTurret);
                if (endAffector.checkBottom()) {
                    state = State.MOVE_V4DOWN;
                }
                break;
            case MOVE_V4DOWN:
                System.out.println("out");
                endAffector.setV4Bar(intakePitch);
                if (endAffector.v4Servo.getCurrentAngle() == intakePitch) {
                    state = State.DOWN;
                }
                break;

            case DOWN:
                // slides.setLength(0.0); BOMB -- Eric
                endAffector.setV4Bar(intakePitch);
                endAffector.setTopTurret(intakeTopTurret);
                endAffector.setBotTurret(intakeBotTurret);
                endAffector.setTopServo(intakeTopServoAngle);
                break;

            case WAIT: // We are boring :(
                break;
        }
        slides.update();
        dunker.update();
    }
}
