package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

// https://www.ctrlaltftc.com/the-pid-controller
public class PID {
    public double p, i, d;
    private double lastError = 0;
    private double integralSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getOut(double error) {
        lastError = error;
        integralSum += error * i * timer.seconds();
        double out = (p * error) + integralSum + (d * (error - lastError) / timer.seconds());
        timer.reset();
        return out;
    }

    public void reset() {
        integralSum = 0;
        timer.reset();
    }

    public void updatePID(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }
}