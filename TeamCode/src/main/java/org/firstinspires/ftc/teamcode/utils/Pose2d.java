package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

public class Pose2d implements Cloneable {
    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y){
        this(x,y,0);
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public boolean isNaN(){
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }

    public double getX(){ return x; }
    public double getY(){
        return y;
    }
    public double getHeading(){
        return heading;
    }

    public double getDistanceFromPoint(Pose2d newPoint) { // distance equation
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
    }

    public double getErrorInX(Pose2d newPoint) { // distance equation
        return Math.abs(x - newPoint.x);
    }

    public double getErrorInY(Pose2d newPoint) { // distance equation
        return Math.abs(y - newPoint.y);
    }

    public void clipAngle() {
        heading = AngleUtil.clipAngle(heading);
    }

    @NonNull
    @Override
    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }
}