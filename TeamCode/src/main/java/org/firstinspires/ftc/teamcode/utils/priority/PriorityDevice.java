package org.firstinspires.ftc.teamcode.utils.priority;

public abstract class PriorityDevice {
    protected final double basePriority, priorityScale;
    protected final String name;
    protected double lastUpdateTime, callLengthMillis;
    boolean isUpdated = false;

    public PriorityDevice(double basePriority, double priorityScale, String name) {
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        this.name = name;
        lastUpdateTime = System.nanoTime();
    }

    protected abstract double getPriority(double timeRemaining);

    protected abstract void update();

    public void resetUpdateBoolean() {
        isUpdated = false;
    }
}
