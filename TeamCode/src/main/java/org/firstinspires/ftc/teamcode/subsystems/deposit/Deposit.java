package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Deposit {

    DepositMath depositMath;
    enum DepositState {
        DOWN,
        EXTEND,
        HOLD,
        DEPOSIT,
        RETRACT
    }
    public DepositState state;

    public Slides slides;
    HardwareQueue hardwareQueue;
    Sensors sensors;

    Pose2d targetBoard;
    double targetY;
    double targetH;

    boolean extend;
    boolean deposit;
    public Deposit(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;
        depositMath = new DepositMath();

        state = DepositState.DOWN;

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

    public void extend() {
        extend = true;
    }
    public void deposit() {
        deposit = true;
    }

    public void update() {
        switch (state) {
            case DOWN:

                if (extend) {
                    //set v4bar to correct position
                    state = DepositState.EXTEND;
                    extend = false;
                }
                break;
            case EXTEND:
                if (/* bucket not in position */ false) {
                    break;
                }

                depositMath.calculate(targetBoard.x-ROBOT_POSITION.x,
                        targetBoard.y-ROBOT_POSITION.y,
                        targetBoard.heading-ROBOT_POSITION.heading,
                        targetH, targetY );
                slides.setLength(depositMath.slideExtension);
                if (Math.abs(depositMath.slideExtension - slides.length) < 1) {
                    state = DepositState.HOLD;
                }
                break;
            case HOLD:
                /* set v4 + bucket position */
                if (/* v4 and bucket ready */ true) {
                    state = DepositState.DEPOSIT;
                }
            case DEPOSIT:
                depositMath.calculate(targetBoard.x-ROBOT_POSITION.x,
                        targetBoard.y-ROBOT_POSITION.y,
                        targetBoard.heading-ROBOT_POSITION.heading,
                        targetH, targetY );
                /*deposit */
                if (/*done depositing*/ true) {
                    //v4bar resting pos
                    state = DepositState.RETRACT;
                }
                break;
            case RETRACT:
                if (/*v4bar not ready*/ false) {
                    break;
                }
                /* retract slides */
                if (/* slides done */ false) {
                    state = DepositState.DOWN;
                }
                break;
        }
    }

    public void teleOp() {

    }
}
