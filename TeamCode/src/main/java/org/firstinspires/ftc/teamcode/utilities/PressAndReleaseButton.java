package org.firstinspires.ftc.teamcode.utilities;

public class PressAndReleaseButton {
    private boolean lastPressed;
    private boolean isTrue;

    public PressAndReleaseButton() {
        lastPressed = false;
    }

    public void periodic(boolean buttonInput) {
        if (buttonInput && !lastPressed) {
            isTrue = true;
            lastPressed = true;
        } else if (buttonInput) {
            isTrue = false;
            lastPressed = true;
        } else {
            isTrue = false;
            lastPressed = false;
        }
    }

    public boolean getIsTrue() {
        return isTrue;
    }
}
