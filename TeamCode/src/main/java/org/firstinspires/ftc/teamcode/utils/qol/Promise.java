package org.firstinspires.ftc.teamcode.utils.qol;

import java.util.ArrayList;

public class Promise {
    private static ArrayList<Integer> waitingPromises = new ArrayList<>();

    private GenericCallback cb = null;
    private boolean done = false;

    /**
     * Multiple .then calls will not work. Don't be a monkey
     * @param cb
     */
    public Promise then(GenericCallback cb) {
        this.cb = cb;
        return this;
    }

    public void call(Object data) {
        done = true;
        if (cb != null) {
            cb.func(data);
        }
    }

    /*
     * Ong I have no clue if this one works
     */
    public static Promise all(Promise[] promises) {
        Promise ret = new Promise();
        int index = waitingPromises.size(); // Storage for callback usage
        waitingPromises.add(promises.length);
        // Store all promise data to send it in the event
        Object[] promiseData = new Object[promises.length];

        for (int i = 0; i < promises.length; i++) {
            int finalI = i; // Hack to use i
            promises[i].then((Object event) -> {
                int value = waitingPromises.get(index) - 1;
                promiseData[finalI] = event;
                if (value <= 0) {
                    waitingPromises.remove(index);
                    /**
                     * WEE OO WEE OO! You can probably shoot yourself in the foot here!
                     * The passed variable is an Object[]. Don't panic. Just cast it to
                     * Object[] and not to anything else. Items in it should be casted
                     * after they are grabbed.
                     */
                    ret.call(promiseData);
                } else {
                    waitingPromises.set(index, value);
                }
            });
        }

        return ret;
    }

    public boolean done() {
        return done;
    }
}
