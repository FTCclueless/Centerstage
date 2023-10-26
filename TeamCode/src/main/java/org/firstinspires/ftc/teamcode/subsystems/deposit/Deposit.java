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
    public EndAffector endAffector;
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
        endAffector = new EndAffector(hardwareMap, hardwareQueue, sensors);
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
        /* Deposit this number of pixels and because its one servo we don't need none of that state yucky yucky (I THINK??) */
    }

    public void update() {
        switch (state) {
            case START_DEPOSIT: // any adjustments initialize here --Kyle
                depositMath.calculate(targetBoard.x - ROBOT_POSITION.x,
                    targetBoard.y - ROBOT_POSITION.y,
                    targetBoard.heading - ROBOT_POSITION.heading,
                    targetH, targetY
                );

                slides.setLength(Math.max(depositMath.slideExtension, Slides.minDepositHeight+1));

                if (slides.length>= Slides.minDepositHeight)
                    state = State.FINISH_DEPOSIT;

                break;

            case FINISH_DEPOSIT: // completion of everything
                endAffector.setV4Bar(depositMath.v4BarPitch);
                endAffector.setBotTurret(depositMath.v4BarYaw);
                endAffector.setTopTurret(targetBoard.heading - ROBOT_POSITION.heading - depositMath.v4BarYaw);

                if (/* mini turret at angle && v4bar is at right angle */ true)
                    state = State.UP;

                break;

            case UP: // We are doing nothing --kyle

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
        }
        slides.update();
    }
}
