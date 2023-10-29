package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.widget.Button;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Deposit {

    DepositMath depositMath;
    enum State {
        START_DEPOSIT,
        FINISH_DEPOSIT,
        START_RETRACT,
        FINISH_RETRACT,
        UP,
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

    public boolean isAuto;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors, boolean isAuto) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        this.isAuto = isAuto;
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
    public void setTargetPlace(double yOffset, double boardHeight) {
        targetY = yOffset;
        targetH = boardHeight;
    }

    // Call this whenever you want! It can be an updating function!
    public void depositAt(double targetH, double targetY) {
        this.targetH = targetH;
        this.targetY = targetY;

        if (state == State.DOWN)
            state = State.START_DEPOSIT;
        if (state == State.UP)
            state = State.START_DEPOSIT;

    }

    public void dunk(int numPixels) {
        numPixels = Math.min(Math.max(0, numPixels), 2);
        if (numPixels == 2) {
            dunker.dunk2();
        }
        else {
            dunker.dunk1();
        }
        /* Deposit this number of pixels and because its one servo we don't need none of that state yucky yucky (I THINK??) */
    }

    public void retract() {
        state = State.START_RETRACT;
    }

    ButtonToggle up = new ButtonToggle();
    ButtonToggle down = new ButtonToggle();
    ButtonToggle left = new ButtonToggle();
    ButtonToggle right = new ButtonToggle();

    public void teleOp(Gamepad gamepad2) {
        xError += gamepad2.right_stick_y*0.1;
        yError += gamepad2.right_stick_x*0.1;
        targetH += gamepad2.right_stick_y * 0.5;

        targetH += (up.isClicked(gamepad2.dpad_up) ? 3 : 0);
        targetH -= (down.isClicked(gamepad2.dpad_down) ? 3 : 0);
        yError -= (left.isClicked(gamepad2.dpad_left) ? 3 : 0);
        yError += (right.isClicked(gamepad2.dpad_right) ? 3 : 0);

        update();
    }

    public void update() {
        switch (state) {
            case START_DEPOSIT: // any adjustments initialize here --Kyle
                if (isAuto){
                     xError = targetBoard.x - ROBOT_POSITION.x;
                     yError =  targetBoard.y - ROBOT_POSITION.y;
                     headingError = targetBoard.heading - ROBOT_POSITION.heading;
                }
                depositMath.calculate(xError,
                    yError,
                    headingError,
                    targetH, targetY
                );

                slides.setLength(Math.max(depositMath.slideExtension, Slides.minDepositHeight+1));

                endAffector.setV4Bar(depositMath.v4BarPitch);

                if (endAffector.checkV4())
                    state = State.FINISH_DEPOSIT;
                break;
            case FINISH_DEPOSIT:
                slides.setLength(Math.max(depositMath.slideExtension, Slides.minDepositHeight+1));
                endAffector.setBotTurret(depositMath.v4BarYaw);
                endAffector.setTopTurret(targetBoard.heading - ROBOT_POSITION.heading - depositMath.v4BarYaw);

                if (endAffector.checkReady()&& slides.length == depositMath.slideExtension)
                    state = State.UP;
                break;

            case UP: // We are doing nothing --kyle

                break;

            case START_RETRACT:
                endAffector.setBotTurret(0);
                endAffector.setTopTurret(Math.PI);
                /* move v4bar servo to minimum value before bricking */

                if (endAffector.checkReady())
                    state = State.FINISH_RETRACT;

                break;

            case FINISH_RETRACT:
                slides.setLength(0);

                endAffector.setV4Bar(EndAffector.intakePitch);
                /* set v4bar to retract angle */

                if (slides.state == Slides.State.READY && endAffector.checkReady())
                    state = State.DOWN;

                break;

            case DOWN: // We are boring :(
                break;
        }
        slides.update();
        dunker.update();
    }
}
