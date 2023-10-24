package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
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
    HardwareQueue hardwareQueue;
    Sensors sensors;

    Pose2d targetBoard;
    double targetY;
    double targetH;

    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        depositMath = new DepositMath();

        state = State.DOWN;

        slides = new Slides(hardwareMap, hardwareQueue, sensors);
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
    }

    public void dunk(int numPixels) {
        numPixels = Math.min(Math.max(0, numPixels), 2);
        /* Deposit this number of pixels and because its one servo we don't need none of that state yucky yucky (I THINK??) */
    }

    public void update() {
        switch (state) {
            case START_DEPOSIT:
                depositMath.calculate(targetBoard.x - ROBOT_POSITION.x,
                    targetBoard.y - ROBOT_POSITION.y,
                    targetBoard.heading - ROBOT_POSITION.heading,
                    targetH, targetY
                );

                slides.setLength(depositMath.slideExtension);
                // set V4Bar length to depositMath.v4BarLength

                if (/* v4Bar length is long enough to not die */ true)
                    state = State.FINISH_DEPOSIT;

                break;

            case FINISH_DEPOSIT:
                // mini turret to depositMath.v4BarYaw

                if (/* mini turret at angle && v4bar is at full length */ true)
                    state = State.UP;

                break;

            case UP: // We are static but are available to adjust at any time :)
                depositMath.calculate(targetBoard.x - ROBOT_POSITION.x,
                    targetBoard.y - ROBOT_POSITION.y,
                    targetBoard.heading - ROBOT_POSITION.heading,
                    targetH, targetY
                );

                slides.setLength(depositMath.slideExtension);
                // set V4Bar length to depositMath.v4BarLength
                // mini turret to depositMath.v4BarYaw

                break;

            case START_RETRACT:
                /* set mini turret to have a value of 0 */
                /* move v4bar servo to minimum value before bricking */

                if (/* both servos ready */ true)
                    state = State.FINISH_RETRACT;

                break;

            case FINISH_RETRACT:
                slides.setLength(0);
                /* set v4bar to retract angle */

                if (slides.state == Slides.State.READY && /* v4bar ready */ false)
                    state = State.DOWN;

                break;

            case DOWN: // We are boring :(
                break;

//            case DOWN:
//
//                if (extend) {
//                    //set v4bar to correct position
//                    state = State.EXTEND;
//                    extend = false;
//                }
//                break;
//            case EXTEND:
//                if (/* bucket not in position */ false) {
//                    break;
//                }
//
//                depositMath.calculate(targetBoard.x-ROBOT_POSITION.x,
//                        targetBoard.y-ROBOT_POSITION.y,
//                        targetBoard.heading-ROBOT_POSITION.heading,
//                        targetH, targetY );
//                slides.setLength(depositMath.slideExtension);
//                if (Math.abs(depositMath.slideExtension - slides.length) < 1) {
//                    state = State.HOLD;
//                }
//                break;
//            case HOLD:
//                /* set v4 + bucket position */
//                if (/* v4 and bucket ready */ true) {
//                    state = State.DEPOSIT;
//                }
//            case DEPOSIT:
//                depositMath.calculate(targetBoard.x-ROBOT_POSITION.x,
//                        targetBoard.y-ROBOT_POSITION.y,
//                        targetBoard.heading-ROBOT_POSITION.heading,
//                        targetH, targetY );
//                /*deposit */
//                if (/*done depositing*/ true) {
//                    //v4bar resting pos
//                    state = State.RETRACT;
//                }
//                break;
//            case RETRACT:
//                if (/*v4bar not ready*/ false) {
//                    break;
//                }
//                /* retract slides */
//                if (/* slides done */ false) {
//                    state = State.DOWN;
//                }
//                break;
        }
    }
}
