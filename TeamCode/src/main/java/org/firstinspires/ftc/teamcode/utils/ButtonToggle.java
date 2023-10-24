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
}