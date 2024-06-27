package org.firstinspires.ftc.teamcode.utils.priority;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;

import java.util.ArrayList;

public class HardwareQueue {
    public ArrayList<PriorityDevice> devices = new ArrayList<>();
    public double targetLoopLength = 0.008; // sets the target loop time in seconds

    public PriorityDevice getDevice(String name){
        for (PriorityDevice device : devices){
            if (device.name.equals(name)){
                return device;
            }
        }
        return null;
    }

    public void addDevice(PriorityDevice device) {
        devices.add(device);
    }

    public void update() {
        for (PriorityDevice device: devices) {
            device.resetUpdateBoolean();
        }

        double bestDevice;
        double loopTime = GET_LOOP_TIME(); // finds loopTime in seconds
        int numUpdates = 0;
        do { // updates the motors while still time remaining in the loop
            int bestIndex = 0;
            bestDevice = devices.get(0).getPriority(targetLoopLength - loopTime);

            // finds motor that needs updating the most
            for (int i = 1; i < devices.size(); i++) { //finding the motor that is most in need of being updated;
                double currentMotor = devices.get(i).getPriority(targetLoopLength - loopTime);
                if (currentMotor > bestDevice) {
                    bestIndex = i;
                    bestDevice = currentMotor;
                }
            }
            if (bestDevice != 0) { // priority # of motor needing update the most
                devices.get(bestIndex).update(); // Resetting the motor priority so that it knows that it updated the motor and setting the motor of the one that most needs it
                numUpdates++;
            }
            loopTime = GET_LOOP_TIME();
        } while (bestDevice > 0 && loopTime <= targetLoopLength);
        // Log.i("numUpdates", numUpdates + "");
    }
}
