package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.util.Log;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector3;

public class DepositMath {
    public final double v4BarLength = 40;
    private final Vector3 slidePos = new Vector3(0,0,0);
    public final double slideAngle = Math.toRadians(60);
    private final Vector3 slideUnit = new Vector3(Math.cos(slideAngle),0,Math.sin(slideAngle));

    public double slideExtension;
    public double v4BarYaw;
    public double v4BarPitch;


    public DepositMath() {
    }

    public void calculate(double xError, double yError, double headingError, double slideHeight, double yOffset) {
        xError += slideHeight * Math.cos(slideAngle);
        yError += yOffset;
        double relX = xError*Math.cos(headingError) + yError*Math.sin(headingError);
        double relY = -xError*Math.sin(headingError) + yError*Math.cos(headingError);

        Vector3 depositPos = new Vector3(relX - slidePos.x, relY - slidePos.y, slideHeight*Math.sin(slideAngle) - slidePos.z);
        Vector3 slideProject = Vector3.project(depositPos, slideUnit);
        Vector3 remainder = Vector3.subtract(depositPos, slideProject);

        slideExtension = slideProject.getMag();
        if (Math.abs(remainder.getMag()) > v4BarLength) {
            Log.e("deposit out of range!!!", "e");
        }
        else {
            double extra = Math.sqrt(Math.pow(v4BarLength, 2) - Math.pow(remainder.getMag(), 2));
            slideExtension -= extra;
            remainder = Vector3.subtract(depositPos, Vector3.mul(slideUnit, slideExtension));
        }
        if (slideExtension > Slides.maxSlidesHeight || slideExtension < 0) {
            Log.e("slide out of range", slideExtension + "");
            slideExtension = Math.min(Math.max(slideExtension,0),Slides.maxSlidesHeight);
        }
        v4BarYaw = Math.atan2(remainder.y,remainder.x);
        v4BarPitch = Math.atan2(remainder.z, Math.sqrt(Math.pow(remainder.x,2) + Math.pow(remainder.y,2)));
    }


}
