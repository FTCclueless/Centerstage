package org.firstinspires.ftc.teamcode.utils;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {
    // queue used to store list so that we get the average
    private final Queue<Double> Dataset = new LinkedList<Double>();
    private final int period;
    private double sum;

    MovingAverage movingAverage_X;
    MovingAverage movingAverage_Y;
    MovingAverage movingAverage_Heading;

    // constructor to initialize period
    public MovingAverage(int period, boolean isSecondInit) {
        this.period = period;
    }

    public MovingAverage(int period) {
        this.period = period;

        // need to have the secondInit or else will cause infinite creation of class
        this.movingAverage_X = new MovingAverage(period, true);
        this.movingAverage_Y = new MovingAverage(period, true);
        this.movingAverage_Heading = new MovingAverage(period, true);
    }

    // function to add new data in the
    // list and update the sum so that
    // we get the new mean
    public void addData(double num)
    {
        sum += num;
        Dataset.add(num);

        // Updating size so that length
        // of data set should be equal
        // to period as a normal mean has
        if (Dataset.size() > period) {
            sum -= Dataset.remove();
        }
    }

    public void addPose2d(Pose2d pose)
    {
        movingAverage_X.addData(pose.getX());
        movingAverage_Y.addData(pose.getY());
        movingAverage_Heading.addData(pose.getHeading());
    }

    // function to calculate mean
    public double getMovingAverageForNum() { return sum / period; }

    public Pose2d getMovingAverageForPose2d() {
        return new Pose2d(
                movingAverage_X.getMovingAverageForNum(),
                movingAverage_Y.getMovingAverageForNum(),
                movingAverage_Heading.getMovingAverageForNum()
        );
    }

}