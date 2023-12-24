package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Release {
    public PriorityServo release;

    public static double intakeAngle = 2.280185;
    public static double holdAngle = 2.280185;
    public static double dunkOneAngle = 2.280185;
    public static double dunkTwoAngle = 2.280185;
    public static double fullyCloseAngle = 2.280185;

    public static double dunkTime = 500;
    public static double timer = 0;

    boolean busy = false;

    public Release(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        release = new PriorityServo(hardwareMap.get(Servo.class, "release"), "release",
                PriorityServo.ServoType.SPEED,
                1,
                0,
                1,
                0,
                false,
                1, 2);
        hardwareQueue.addDevice(release);
    }

    public void fullyClose() {
        release.setTargetAngle(fullyCloseAngle, 1);
    }

    public void intake() {
        release.setTargetAngle(intakeAngle, 1);
    }

    public void hold() {
        release.setTargetAngle(holdAngle, 1);
    }

    public void releaseOne() {
        release.setTargetAngle(dunkOneAngle, 1);
        Globals.NUM_PIXELS--;

        if (!busy) {
            dunkTime = 250;
            timer = System.currentTimeMillis();
            busy = true;
        }
    }

    public void releaseTwo() {
        release.setTargetAngle(dunkTwoAngle, 1);
        Globals.NUM_PIXELS-=2;

        if (!busy) {
            dunkTime = 500;
            timer = System.currentTimeMillis();
            busy = true;
        }
    }

    public boolean readyToRetract() {
        return Globals.NUM_PIXELS == 0 && !busy;
    }

    public void update() {
        if (System.currentTimeMillis() - timer >= dunkTime) {
            busy = false;
        }
    }

    public boolean inPosition() {
        return release.inPosition();
    }
}
