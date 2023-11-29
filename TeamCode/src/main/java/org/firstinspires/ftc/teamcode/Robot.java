package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.airplane.Airplane;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Dunker;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.tests.AutoPathTester;
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
            Log.e("isbusy: ", drivetrain.isBusy() + "");
            update();
        }
    }

    public void goToPoint(Pose2d pose, LinearOpMode opMode) {
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start <= 5000 && drivetrain.isBusy()) {
            TelemetryUtil.packet.fieldOverlay().setStroke("red");
            TelemetryUtil.packet.fieldOverlay().strokeCircle(pose.x, pose.y, 3);
            drivetrain.goToPoint(pose);
            update();
        }
        Log.e("go to point x error", (pose.x - drivetrain.localizer.x) + "");
        Log.e("go to point y error", (pose.y - drivetrain.localizer.y) + "");
        Log.e("go to point heading error", (pose.heading - drivetrain.localizer.heading) + "");
    }

    public void goToPoint(double x, double y, double heading, LinearOpMode opMode) {
        this.goToPoint(new Pose2d(x, y, heading), opMode);
    }

    public void depositAt(double targetH, double targetY) {

        deposit.inPlace(); // temporary since we're starting depositAt when we're alredy ready for lgm1 --Kyle
        deposit.depositAt(targetH, targetY);

        while (deposit.state != Deposit.State.WAIT_DUNK) {
            update();
        }
    }

    public void dunk(int numpix) {
        deposit.inPlace();
        while (!deposit.endAffector.checkReady() && deposit.slides.isBusy()) { // TODO: Might be a bomb -- Eric
            System.out.println("first bomb");
            update();
        }
        deposit.dunk(numpix);
        while (deposit.dunker.dunkState != Dunker.DunkState.CHILL) {
            System.out.println("Bomb 2");
            update();
        }
    }

}
