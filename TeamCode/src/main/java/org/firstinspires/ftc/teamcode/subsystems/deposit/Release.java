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

    public static double transferAngle = -1.1009847;
    public static double preGrabAngle = -1.0061777;
    public static double closeAngle = -0.48015169;
    public static double dunkOneAngle = -0.8073888;
    public static double dunkTwoAngle = -1.10098;

    public static double dunkTime = 250; // TODO: CHANGE LINE 59 AND 62
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

    public void preGrab() {
        release.setTargetAngle(preGrabAngle, 1);
    }

    public boolean isBusy() {return busy;}

    public void releaseOne() {
        if (!busy) {
            if (Globals.NUM_PIXELS == 2) {
                release.setTargetAngle(dunkOneAngle, 1);
                dunkTime = 250;
            } else {
                release.setTargetAngle(dunkTwoAngle, 1);
                dunkTime = 125;
            }
            Globals.NUM_PIXELS--;
            busy = true;
            timer = System.currentTimeMillis();
        }
    }

    boolean releasingTwo = false;
    public void releaseTwo() {
        releaseOne();
        releasingTwo = true;
    }

    public void releaseTwoFORCED() {
        release.setTargetAngle(dunkTwoAngle, 1);
    }

    public boolean readyToRetract() {
        return Globals.NUM_PIXELS <= 0 && !busy;
    }

    public void update() {
        if (System.currentTimeMillis() - timer >= dunkTime) {
            busy = false;
            if (releasingTwo) {
                releaseOne();
                releasingTwo = false;
            }
        }
    }

    public boolean inPosition() {
        return release.inPosition();
    }
}
