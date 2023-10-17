package org.firstinspires.ftc.teamcode.subsystems.deposit;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Deposit {

    DepositMath depositMath;
    enum DepositState {
        DOWN,
        EXTEND,
        DEPOSIT,
        RETRACT
    }
    DepositState state;

    public Slides slides;
    HardwareQueue hardwareQueue;
    Sensors sensors;

    Pose2d targetBoard;
    double targetY;
    double targetH;

    boolean launch;
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

    public void launch() {
        launch = true;
    }

    public void update() {
        switch (state) {
            case DOWN:
                if (launch) {

                    state = DepositState.EXTEND;
                    launch = false;
                }
                break;
            case EXTEND:
                depositMath.calculate(targetBoard.x-ROBOT_POSITION.x,
                        targetBoard.y-ROBOT_POSITION.y,
                        targetBoard.heading-ROBOT_POSITION.heading,
                        targetH, targetY );
                /* extend slides */
                if (/*slides extended*/ true) {
                    state = DepositState.DEPOSIT;
                }
                break;
            case DEPOSIT:
                depositMath.calculate(targetBoard.x-ROBOT_POSITION.x,
                        targetBoard.y-ROBOT_POSITION.y,
                        targetBoard.heading-ROBOT_POSITION.heading,
                        targetH, targetY );
                /*set deposit position + continue extending slides*/
                if (/*done depositing*/ true) {
                    //v4bar resting pos
                    state = DepositState.RETRACT;
                }
                break;
            case RETRACT:
                /* retract slides */
                if (/* slides done */ false) {
                    state = DepositState.DOWN;
                }
                break;
        }
    }
}
