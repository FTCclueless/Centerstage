package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.provider.Settings;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
public class Release {
    public PriorityServo release;

    public static double transferAngle = 1.13750;
    public static double closeAngle = 1.849141;
    public static double dunkOneAngle = 1.4512;
    public static double dunkTwoAngle = 1.13750;

    public static double dunkTime = 500;
    public static double timer = 0;

    boolean busy = false;

    public Release(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        release = new PriorityServo(hardwareMap.get(Servo.class, "release"), "release",
                PriorityServo.ServoType.PRO_MODELER,
                1,
                0,
                1,
                0.36,
                false,
                1, 2);
        hardwareQueue.addDevice(release);
    }

    public void intake() {
        release.setTargetAngle(transferAngle, 1);
    }

    public void close() {
        release.setTargetAngle(closeAngle, 1);
    }

    public void releaseOne() {
        Log.e("Globals.NUM_PIXELS", Globals.NUM_PIXELS + "");
        Log.e("busy?", busy + "");
        Log.e("readyToRetract", readyToRetract() + "");

        if (Globals.NUM_PIXELS == 2) {
            release.setTargetAngle(dunkOneAngle, 1);
        } else {
            release.setTargetAngle(dunkTwoAngle, 1);
        }
        Globals.NUM_PIXELS--;

        if (!busy) {
            dunkTime = 250;
            timer = System.currentTimeMillis();
            busy = true;
        }
    }

    public void releaseTwo() {
        release.setTargetAngle(dunkTwoAngle, 1);
        Globals.NUM_PIXELS = 0;

        if (!busy) {
            dunkTime = 500;
            timer = System.currentTimeMillis();
            busy = true;
        }
    }

    public boolean readyToRetract() {
        return Globals.NUM_PIXELS <= 0 && !busy;
    }

    public void update() {
        if (System.currentTimeMillis() - timer >= dunkTime) {
            busy = false;
        }
        if (Globals.NUM_PIXELS == 1 && !busy) {
            close();
        }
    }

    public boolean inPosition() {
        return release.inPosition();
    }
}
