package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.airplane.Airplane;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Robot {
    private HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;

    public final Sensors sensors;
    public final Drivetrain drivetrain;
    public final Deposit deposit;
    public final Intake intake;
    public final Airplane airplane;
    public final Hang hang;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        hardwareQueue = new HardwareQueue();

        sensors = new Sensors(hardwareMap, hardwareQueue);
        drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors);
        deposit = new Deposit(hardwareMap, hardwareQueue, sensors);
        intake = new Intake(hardwareMap, hardwareQueue, sensors);
        airplane = new Airplane(hardwareMap, hardwareQueue);
        hang = new Hang(hardwareMap, hardwareQueue);

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
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());
        TelemetryUtil.sendTelemetry();
    }

    public void followSpline(Spline spline, LinearOpMode opMode) {
        drivetrain.setCurrentPath(spline);
        while(drivetrain.isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }

    public void goToPoint(Pose2d pose, LinearOpMode opMode) {
        drivetrain.goToPoint(pose);
        while(Math.abs(pose.x-ROBOT_POSITION.x) < 2 && Math.abs(pose.y-ROBOT_POSITION.y) < 2 && Math.abs(pose.heading - ROBOT_POSITION.heading) < Math.toRadians(2) && opMode.opModeIsActive()) {
            update();
        }
    }
}
