package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Robot {
    HardwareMap hardwareMap;

    public Sensors sensors;
    public Drivetrain drivetrain;

    public HardwareQueue hardwareQueue = new HardwareQueue();


    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors);
    }

    public void update() {
        START_LOOP();
        updateSubsystems();

    }

    public void updateSubsystems() {
        drivetrain.update();

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

}