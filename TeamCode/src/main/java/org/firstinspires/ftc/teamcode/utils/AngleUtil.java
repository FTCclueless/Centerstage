package org.firstinspires.ftc.teamcode.utils;

public class AngleUtil {
    public static double clipAngle(double angle) {
        while (Math.abs(angle) > Math.PI) {
            angle -= Math.PI * 2.0 * Math.signum(angle);
        }
        return angle;
    }

    public static double toRadians(double angle) {
        return angle * (Math.PI/180);
    }

    public static double toDegrees(double angle) {
        return angle * (180/Math.PI);
    }
}
