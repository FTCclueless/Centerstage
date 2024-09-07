package org.firstinspires.ftc.teamcode.utils;

public class Encoder {
    public double ticksToInches;
    public int lastVal;
    public int currentVal;
    public double scaleFactor;
    public double x;
    public double y;

    public Encoder (Pose2d point, double scaleFactor) {
        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.984; //for 50mm wheels
        ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;
        x = point.getX();
        y = point.getY();
        currentVal = 0;
        lastVal = currentVal;
        this.scaleFactor = scaleFactor;
    }

    public void update(int currentPos) {
        lastVal = currentVal;
        currentVal = currentPos;
    }

    public double getDelta() {
        return (double)(currentVal-lastVal)*ticksToInches*scaleFactor;
    }
    public double getCurrentDist(){
        return (double)(currentVal)*ticksToInches*scaleFactor;
    }
}
