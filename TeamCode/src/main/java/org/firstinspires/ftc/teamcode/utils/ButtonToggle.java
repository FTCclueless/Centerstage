package org.firstinspires.ftc.teamcode.utils;

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

    long startOfButtonHeld;
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
}