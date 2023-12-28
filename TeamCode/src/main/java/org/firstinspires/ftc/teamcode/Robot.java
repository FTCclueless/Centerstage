package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.airplane.Airplane;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.droppers.Droppers;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hangActuation.HangActuation;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    public HardwareQueue hardwareQueue;

    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final Deposit deposit;
    public final Intake intake;
    public final Airplane airplane;
    public final Hang hang;
    public final Droppers droppers;
    public final HangActuation hangActuation;
    public final Vision vision;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Vision vision) {
        hardwareQueue = new HardwareQueue();
        this.vision = vision;

        sensors = new Sensors(hardwareMap, hardwareQueue);

        if (vision != null) {
            drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors, vision);
        } else {
            drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors);
        }

        deposit = new Deposit(hardwareMap, hardwareQueue, sensors, this);
        intake = new Intake(hardwareMap, hardwareQueue, sensors, this);
        airplane = new Airplane(hardwareMap, hardwareQueue);
        hang = new Hang(hardwareMap, hardwareQueue, sensors);
        droppers = new Droppers(hardwareMap, hardwareQueue);
        hangActuation = new HangActuation(hardwareMap, hardwareQueue);

        TelemetryUtil.setup();
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
        updateTelemetry();
    }

    private void updateSubsystems() {
        hardwareQueue.update();
        sensors.update();

        drivetrain.update();
        deposit.update();
        intake.update();
        airplane.update();
        hang.update();
        droppers.update();
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
    }

    public void goToPoint(Pose2d pose, LinearOpMode opMode, boolean finalAdjustment) {
        long start = System.currentTimeMillis();
        drivetrain.goToPoint(pose, finalAdjustment); // need this to start the process so thresholds don't immediately become true
        while(opMode.opModeIsActive() && System.currentTimeMillis() - start <= 5000 && drivetrain.isBusy()) {
            update();
        }
    }

    public void goToPoint(double x, double y, double heading, LinearOpMode opMode, boolean finalAdjustment) {
        this.goToPoint(new Pose2d(x, y, heading), opMode, finalAdjustment);
    }

    public void depositAt(double targetH, double targetX) {
        deposit.depositAt(targetH, targetX);

        while (deposit.state != Deposit.State.DEPOSIT) {
            update();
        }
    }

    public void releaseOne() {
        deposit.releaseOne();
        while (deposit.state != Deposit.State.RETRACT) {
            update();
        }
    }

    public void releaseTwo() {
        deposit.releaseTwo();
        while (deposit.state != Deposit.State.START_RETRACT) {
            update();
        }
    }
}
