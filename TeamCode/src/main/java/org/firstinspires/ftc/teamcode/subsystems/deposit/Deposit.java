package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

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
        FINISH_RETRACT,
        DOWN,
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
    private double v4barClipThreshold = Math.toRadians(135);
    private static double intakePitch = Math.toRadians(225); //todo
    public static double slidesV4Thresh = 4; //todo

    boolean inPlace = false;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        depositMath = new DepositMath();

        state = State.DOWN;

        slides = new Slides(hardwareMap, hardwareQueue, sensors);
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
        dunker = new Dunker(hardwareMap, hardwareQueue, sensors);

        endAffector.setV4Bar(intakePitch);
        endAffector.setBotTurret(0);
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
        switch (state) {
            case START_DEPOSIT: // any adjustments initialize here --Kyle
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

                    depositMath.calculate(
                        xError,
                        yError,
                        headingError,
                        targetH, targetY
                    );
                }
                slides.setLength(depositMath.slideExtension);

                if (slides.getLength() > slidesV4Thresh)
                    state = State.FINISH_DEPOSIT; // skipping two cases because no need anymore --kyle

                break;
            case MOVE_V4UP:
                endAffector.setV4Bar(Math.toRadians(90));
                if (endAffector.v4Servo.getCurrentAngle() <= v4barClipThreshold)
                    state = State.EXTEND_ROTATE180;
                break;

            case EXTEND_ROTATE180:
                endAffector.setBotTurret(Math.toRadians(0));
                if (endAffector.botTurret.getCurrentAngle() < Math.toRadians(5)) {
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

                    depositMath.calculate(
                            xError,
                            yError,
                            headingError,
                            targetH, targetY
                    );
                }

                slides.setLength(depositMath.slideExtension);
                //endAffector.setBotTurret(depositMath.v4BarYaw);  no adjustments in auto
                endAffector.setV4Bar(depositMath.v4BarPitch);

                if (depositMath.v4BarPitch < 0) {
                    endAffector.setBotTurret(0);
                    Log.e("v4bar too low", "E");
                }

                /*if (Globals.RUNMODE == RunMode.TELEOP) {
                    endAffector.setTopTurret(-depositMath.v4BarYaw);
                } else {
                    endAffector.setTopTurret(targetBoard.heading - ROBOT_POSITION.heading - depositMath.v4BarYaw);
                } still not being used since unnecessary */

                endAffector.setV4Bar(depositMath.v4BarPitch);

                break;
            case WAIT_DUNK:
                if (dunker.dunkState == Dunker.DunkState.CHILL)
                    state = State.START_RETRACT;
                break;

            case START_RETRACT:
                slides.setLength(slides.getLength());
                endAffector.setBotTurret(0);
                endAffector.setTopTurret(Math.toRadians(180));
                endAffector.setV4Bar(Math.PI/2);
                /* move v4bar servo to minimum value before bricking */

                if (endAffector.v4Servo.getCurrentAngle() == Math.PI/2)
                    state = State.FINISH_RETRACT; //skipping for same reason --kyle

                break;
            case RETRACT_ROTATE180:
                endAffector.setBotTurret(Math.PI);
                if (endAffector.botTurret.getCurrentAngle() >= Math.toRadians(175) ) {
                    state = State.MOVE_V4DOWN;
                }
                break;
            case MOVE_V4DOWN:
                endAffector.setV4Bar(intakePitch);
                slides.setLength(slidesV4Thresh);
                if (endAffector.checkV4())
                    state = State.FINISH_RETRACT;
                break;

            case FINISH_RETRACT:
                slides.setLength(0);

                endAffector.setV4Bar(intakePitch);
                /* set v4bar to retract angle */

                if (!slides.isBusy() && endAffector.checkReady()) {
                    inPlace = false; // this might also be a temporary bomb rn --Kyle
                    state = State.DOWN;
                }

                break;

            case DOWN: // We are boring :(
                break;
        }
        slides.update();
        dunker.update();
    }
}
