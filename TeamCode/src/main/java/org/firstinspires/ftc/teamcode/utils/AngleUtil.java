package org.firstinspires.ftc.teamcode.utils;

public class AngleUtil {
    public static double clipAngle(double angle) {
        while (Math.abs(angle) > Math.PI) {
            angle -= Math.PI * 2.0 * Math.signum(angle);
        }
        return angle;
    }
}
