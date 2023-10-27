package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {
    // queue used to store list so that we get the average
    private final ArrayList<Double> Dataset = new ArrayList<Double>();
    private final int period;
    private double sum;

    // constructor to initialize period
    public MovingAverage(int period) {
        this.period = period;
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
            sum -= Dataset.remove(0);
        }
    }

    public void updateValsRetroactively(double val){
        sum -= val*Dataset.size();
        for (int i = 0; i < Dataset.size(); i ++){
            Dataset.set(i,Dataset.get(i) - val);
        }
    }

    // function to calculate mean
    public double getMovingAverageForNum() {
        return sum / Dataset.size();
    }
}