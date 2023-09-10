package org.firstinspires.ftc.teamcode.utils.qol;

import java.util.ArrayList;
import java.util.HashMap;

public class Subsystem {
    private final ArrayList<GenericCallback> genericCallbacks = new ArrayList<>();
    private final ArrayList<TransitionCallback> transitionCallbacks = new ArrayList<>();

    private final ArrayList<Object> entranceCallbacksStates = new ArrayList<>();
    private final ArrayList<EntranceCallback> entranceCallbacks = new ArrayList<>();
    
    private final ArrayList<Object> exitCallbackStates = new ArrayList<>();
    private final ArrayList<ExitCallback> exitCallbacks = new ArrayList<>();
    
    private final HashMap<Object, NoParamCallback> updateCallbacks = new HashMap<>();
    
    public Object state;
    private Object setState;
    
    public Subsystem(Object state) {
        // Avoid garbage
        this.state = state;
        this.setState = state;
    }
    
    protected void switchState(Object data, Object state) {
        for (TransitionCallback cb : transitionCallbacks) {
            cb.func(data, this.state, state);
        }
        
        for (int i = 0; i < entranceCallbacksStates.size(); i++) {
            if (entranceCallbacksStates.get(i) == state) {
                entranceCallbacks.get(i).func(data);
            }
        }
        
        for (int i = 0; i < exitCallbackStates.size(); i++) {
            if (exitCallbackStates.get(i) == this.state) {
                exitCallbacks.get(i).func(data);
            }
        }

        this.state = state;
    }
    
    // Huddy kim copium
    protected void transition(Object data, boolean condition, Object state) {
        if (condition) {
            switchState(data, state);
        }
    }

    public Subsystem addGenericCallback(GenericCallback cb) {
        genericCallbacks.add(cb);
        return this;
    }

    public void callGenericCallback(Object data) {
        for (GenericCallback cb : genericCallbacks) {
            cb.func(data);
        }
    }

    public Subsystem onStateChange(TransitionCallback cb) {
        transitionCallbacks.add(cb);
        return this;
    }

    public Subsystem onEnter(EntranceCallback cb) {
        entranceCallbacksStates.add(setState);
        entranceCallbacks.add(cb);
        return this;
    }

    public void onEnter(Object state, EntranceCallback cb) {
        entranceCallbacksStates.add(state);
        entranceCallbacks.add(cb);
    }

    public void onExit(Object state, ExitCallback cb) {
        exitCallbackStates.add(state);
        exitCallbacks.add(cb);
    }

    public Subsystem onExit(ExitCallback cb) {
        exitCallbackStates.add(setState);
        exitCallbacks.add(cb);
        return this;
    }

    protected Subsystem addState(Object state, NoParamCallback cb) {
        updateCallbacks.put(state, cb);
        setState = state;
        return this;
    }

    public void update() {
        if (updateCallbacks.containsKey(state)) {
            updateCallbacks.get(state).func();
        }
    }
}