package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

public class ButtonToggle {
    boolean lastButton = false;
    boolean isToggled = false;

    public ButtonToggle() {}

    public boolean isClicked(boolean button) {
        boolean a = button && !lastButton;
        lastButton = button;
        return a;
    }

    public boolean isToggled(boolean button) {
        if (isClicked(button)) {
            isToggled = !isToggled;
        }
        return isToggled;
    }

    long startOfButtonHeld = System.currentTimeMillis();
    public boolean isHeld(boolean button, double milliseconds) {
        if (button) {
            if (System.currentTimeMillis() - startOfButtonHeld > milliseconds) {
                return true;
            }
        } else {
            startOfButtonHeld = System.currentTimeMillis();
        }
        return false;
    }

    boolean isBeingHeld = false;
    public boolean isReleased(boolean button) {
        if (isBeingHeld && !button) {
            isBeingHeld = false;
            return true;
        }
        isBeingHeld = button;
        return false;
    }

    boolean previousLoopIsHeld = false;
    public boolean releasedAndNotHeldPreviously(boolean button, double holdTime) {
        boolean isReleased = isReleased(button);
        if (isReleased && !previousLoopIsHeld) {
            Log.e("isReleased", isReleased + "");
            previousLoopIsHeld = isHeld(button, holdTime);
            return true;
        }

        previousLoopIsHeld = isHeld(button, holdTime);
        return false;
    }

}