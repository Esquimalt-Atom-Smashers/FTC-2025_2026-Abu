package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
@Config
public abstract class RobotUtility {
    public static final Pose2d RED_RESET_POS = new Pose2d((72 - 7),-(72 - 7),Math.toRadians(90));
    public static final Pose2d BLUE_RESET_POS = new Pose2d((72 - 7),(72 - 7),Math.toRadians(270));

    private final static double DEAD_ZONE = 0.05;

    public static double deadZoneJoyStick(double joystickInput) {
        if (Math.abs(joystickInput) < DEAD_ZONE ) {
            joystickInput = 0;
        }
        return joystickInput;
    }
}
