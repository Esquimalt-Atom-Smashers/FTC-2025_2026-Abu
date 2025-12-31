package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Property {
    // ---------- ARM PROPERTIES ---------- //
    // AUTO POSITIONS
    // ----------- DRIVE PROPERTIES ---------- //
    public static volatile double STRAFE_OFFSET = 1.1;
    // ----------- AUTO PID / TOLERANCE PROPERTIES ---------- //
    public static volatile double DRIVE_GAIN  = 0.027;
    public static volatile double STRAFE_GAIN = 0.04;
    public static volatile double TURN_GAIN   = 0.03;

    public static volatile double DRIVE_D  = 0.0025;
    public static volatile double STRAFE_D = 0.001;
    public static volatile double TURN_D   = 0.0013;

    // ----------- AUTO WAIT TIMES------------ //

    public static volatile int INITIAL_SLEEP_MS = 4000;
    public static volatile int STEP_1_SLEEP_MS  = 1000;
    public static volatile int STEP_2_SLEEP_MS  = 800;
    public static volatile int STEP_3_SLEEP_MS  = 2000;
}
