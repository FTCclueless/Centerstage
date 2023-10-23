package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Robot {
    HardwareMap hardwareMap;

    public final Sensors sensors;
    public final Drivetrain drivetrain;
    //public final Deposit deposit;

    public HardwareQueue hardwareQueue = new HardwareQueue();


    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        sensors = new Sensors(hardwareMap, hardwareQueue);
        drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors);
        //deposit = new Deposit(hardwareMap, hardwareQueue, sensors);

        TelemetryUtil.setup();
    }

    public void autoIntake(Spline spline) {
        drivetrain.setCurrentPath(spline);
        // turn on intake monko
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
        updateTelemetry();
    }

    public void updateSubsystems() {
        drivetrain.update();
        //depsit.update();

        sensors.update();
        hardwareQueue.update();
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());

        updateDashboard();

        TelemetryUtil.sendTelemetry();
    }

    public void updateDashboard() {
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        Pose2d poseEstimate = drivetrain.getPoseEstimate();

        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
        DashboardUtil.drawSampledPath(fieldOverlay, drivetrain.getCurrentPath());
    }

    public void followSpline(Spline spline, LinearOpMode opMode) {
        drivetrain.setCurrentPath(spline);
        while(drivetrain.isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }
}
