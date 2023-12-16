package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
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
    Robot robot;
    Sensors sensors;

    Pose2d targetBoard;
    double targetY;
    double targetH;

    double xError = 5;
    double yError = 5;
    double headingError = 0;
    double xOffset = 0;
    public static double downPitch = -1.08; // regular down intake pitch for arm
    public static double transferPitch = -1.28; //2.0900 stalling pitch for arm
    public static double slidesV4Thresh = 12;
    public static double upPitch = 1.38;
    public static double intakeTopTurret = 0.056;
    public static double intakeTopServoAngle = 1.115;
    public static double intakeBotTurret = 3.57;

    public static double power = 0.9;

    public static double interpolationDist = 3;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, Robot robot) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.robot = robot;
        depositMath = new DepositMath();

        state = State.DOWN;

        slides = new Slides(hardwareMap, hardwareQueue, sensors);
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
        dunker = new Dunker(hardwareMap, hardwareQueue, sensors);
    }

    public void setTargetBoard(Pose2d targetBoard) {
        this.targetBoard = targetBoard;
    }

    // Call this whenever you want! It can be an updating function!
    public void depositAt(double targetH, double targetY, double xOffset) {
        this.targetH = targetH;
        this.targetY = targetY;
        this.xOffset = xOffset;

        if (state == State.DOWN)
            state = State.START_DEPOSIT;
    }

    public void depositAt(double targetH, double targetY) {
        this.depositAt(targetH, targetY, 4);
    }

    public void resetXOffset() {
        xOffset = 0;
    }

    public void dunk(int numPixels) {
        numPixels = Math.min(Math.max(0, numPixels), 2);
        dunker.dunk2(); // TODO
        state = State.WAIT_DUNK;
        /* Deposit this number of pixels and because its one servo we don't need none of that state yucky yucky (I THINK??) */
    }

    public void retract() {
        state = State.START_RETRACT;
    }

    public boolean checkReady() {
        return endAffector.checkReady() && !slides.isBusy();
    }

    public void update() {
        TelemetryUtil.packet.put("depoState", state);
        switch (state) {
            case START_DEPOSIT: // any adjustments initialize here --Kyle
                dunker.lock();
                if (Globals.RUNMODE == RunMode.TELEOP) {
                    depositMath.calculate(
                        xOffset,
                        0,
                        0,
                        targetH, targetY
                    );
                } else {
                    xError = targetBoard.x - ROBOT_POSITION.x + xOffset;
                    yError = targetBoard.y - ROBOT_POSITION.y;
                    headingError = targetBoard.heading - AngleUtil.clipAngle(ROBOT_POSITION.heading+Math.PI);

                    depositMath.calculate(
                        xError,
                        yError,
                        headingError,
                        targetH, targetY
                    );
                }
                slides.setLength(Math.max(depositMath.slideExtension, slidesV4Thresh));

                Log.e("targetY START", targetY + "");

                if (slides.getLength() > slidesV4Thresh-4)
                    state = State.MOVE_V4UP;

                break;
            case MOVE_V4UP:
                endAffector.v4Servo.setTargetAngle(upPitch,power);
                if (endAffector.v4Servo.inPosition())
                    state = State.EXTEND_ROTATE180;
                break;

            case EXTEND_ROTATE180:
                endAffector.botTurret.setTargetAngle(depositMath.v4BarYaw * 40/36.0,power);
                endAffector.topTurret.setTargetAngle(-depositMath.v4BarYaw, 1.0);
                Log.e("(depositMath.v4BarYaw * 40/36.0)", (depositMath.v4BarYaw * 40/36.0) + "");
                if (endAffector.botTurret.inPosition()) {
                    state = State.FINISH_DEPOSIT;
                }
                break;
            case FINISH_DEPOSIT: // Also our update state -- Eric
                if (Globals.RUNMODE == RunMode.TELEOP) {
                    depositMath.calculate(
                            xOffset,
                            0,
                            0, //targetBoard.heading - AngleUtil.clipAngle(ROBOT_POSITION.heading+Math.PI),
                            targetH, targetY
                    );
                } else {
                    xError = targetBoard.x - ROBOT_POSITION.x + xOffset;
                    yError = targetBoard.y - ROBOT_POSITION.y;
                    headingError = targetBoard.heading - AngleUtil.clipAngle(ROBOT_POSITION.heading+Math.PI);

                    TelemetryUtil.packet.put("xError", xError);
                    TelemetryUtil.packet.put("yError", yError);
                    TelemetryUtil.packet.put("headingError", headingError);

                    depositMath.calculate(
                            xError,
                            yError,
                            headingError,
                            targetH, targetY
                    );
                    Log.e("targetY END", targetY + "");
                }
                TelemetryUtil.packet.put("v4Yaw", depositMath.v4BarYaw);


                slides.setLength(depositMath.slideExtension);
                endAffector.botTurret.setTargetAngle(depositMath.v4BarYaw * 40.0/36,power); //scuffed kinda (gear ratio) --kyle
                endAffector.v4Servo.setTargetAngle(depositMath.v4BarPitch,power);
                endAffector.topServo.setTargetAngle(-depositMath.v4BarPitch,1.0);

                if (depositMath.v4BarPitch < 0) {
                    //endAffector.setBotTurret(0);
                    Log.e("v4bar too low", "E");
                }

                if (Globals.RUNMODE == RunMode.TELEOP) {
                    endAffector.topTurret.setTargetAngle(-depositMath.v4BarYaw,1.0);
                } else {
                    endAffector.topTurret.setTargetAngle(targetBoard.heading - AngleUtil.clipAngle(ROBOT_POSITION.heading+Math.PI) - depositMath.v4BarYaw,1.0);
                }

                break;
            case WAIT_DUNK:
                if (!dunker.busy()) {
                    state = State.START_RETRACT;
                }
                break;

            case START_RETRACT:
                //endAffector.setBotTurret(0);
                slides.setLength(slidesV4Thresh);
                endAffector.botTurret.setTargetAngle(0.0,power);
                //endAffector.topTurret.setTargetAngle(intakeTopTurret,power);
                /* move v4bar servo to minimum value before bricking */

                if (endAffector.botTurret.inPosition()) {
                    endAffector.v4Servo.setTargetAngle(upPitch,power);
                    if (endAffector.v4Servo.inPosition()) {
                        state = State.RETRACT_ROTATE180;
                    }
                }
                break;
            case RETRACT_ROTATE180:
                endAffector.botTurret.setTargetAngle(intakeBotTurret,power);
                if (endAffector.botTurret.inPosition()) {
                    state = State.MOVE_V4DOWN;
                }
                break;
            case MOVE_V4DOWN:
                System.out.println("out");
                endAffector.v4Servo.setTargetAngle(downPitch,power/2);
                if (endAffector.v4Servo.inPosition()) {
                    state = State.DOWN;
                }
                break;

            case DOWN:
                slides.setLength(0.0);

                if (robot.intake.state == Intake.State.ON && sensors.getSlidesPos() < 3) {
                    endAffector.v4Servo.setTargetAngle(transferPitch,power);
                } else {
                    endAffector.v4Servo.setTargetAngle(downPitch,power);
                }

                endAffector.topTurret.setTargetAngle(intakeTopTurret,1.0);
                endAffector.botTurret.setTargetAngle(intakeBotTurret,power);
                endAffector.topServo.setTargetAngle(intakeTopServoAngle,1.0);
                dunker.intake();
                break;

            case WAIT: // We are boring :(
                break;
        }
        slides.update();
        dunker.update();

        TelemetryUtil.packet.put("v4ServoAngle", endAffector.v4Servo.getCurrentAngle());
        TelemetryUtil.packet.put("v4ServoTarget", endAffector.v4Servo.getTargetPosition());
    }
}
