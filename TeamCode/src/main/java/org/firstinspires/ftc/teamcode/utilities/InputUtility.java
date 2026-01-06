package org.firstinspires.ftc.teamcode.utilities;

public abstract class InputUtility {
    private final static double DEAD_ZONE = 0.05;

    public static double deadZoneJoyStick(double joystickInput) {
        if (Math.abs(joystickInput) < DEAD_ZONE ) {
            joystickInput = 0;
        }
        return joystickInput;
    }
}
