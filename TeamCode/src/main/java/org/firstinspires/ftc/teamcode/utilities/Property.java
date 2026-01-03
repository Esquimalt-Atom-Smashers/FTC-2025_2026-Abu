package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Property {
    //GENERAL SETTINGS
    public static volatile double TELEMETRY_UPDATE_TIME = 1.0;

    // AUTO POSITIONS
    public static volatile double RED_FAR_X = 65;
    public static volatile double RED_FAR_Y = 16;
    public static volatile double RED_FAR_HEADING = 180;

    public static volatile double RED_CLOSE_X = -72 + 19.5;
    public static volatile double RED_CLOSE_Y = 72 - 18;
    public static volatile double RED_CLOSE_HEADING = 53.5;

    public static volatile double BLUE_FAR_X = 72 - 7;
    public static volatile double BLUE_FAR_Y = -16;
    public static volatile double BLUE_FAR_HEADING = 180;

    public static volatile double BLUE_CLOSE_X = -72 + 21.5;
    public static volatile double BLUE_CLOSE_Y = -72 + 17.5;
    public static volatile double BLUE_CLOSE_HEADING = 143.65;

    public static volatile double FAR_SHOOT_RPM = 3500;
    public static volatile double CLOSE_SHOOT_RPM = 3100;

    public static volatile double RED_FAR_SHOOT_X = 55;
    public static volatile double RED_FAR_SHOOT_Y = 18;
    public static volatile double RED_FAR_SHOOT_HEADING = 72;

    public static volatile double RED_THIRD_INTAKE_P1_X = 35;
    public static volatile double RED_THIRD_INTAKE_P1_Y = 30;
    public static volatile double RED_THIRD_INTAKE_P1_HEADING = 90;

    public static volatile double RED_THIRD_INTAKE_P2_X = 35;
    public static volatile double RED_THIRD_INTAKE_P2_Y = 43;
    public static volatile double RED_THIRD_INTAKE_P2_HEADING = 90;

    public static volatile double RED_LOAD_INTAKE_P1_X = 44;
    public static volatile double RED_LOAD_INTAKE_P1_Y = 55;
    public static volatile double RED_LOAD_INTAKE_P1_HEADING = 0;

    public static volatile double RED_LOAD_INTAKE_P2_X = 60;
    public static volatile double RED_LOAD_INTAKE_P2_Y = 55;
    public static volatile double RED_LOAD_INTAKE_P2_HEADING = 0;

    //flywheel control
    public static volatile double TOLERANCE = 100;
    public static volatile double kV = 0.000215;

    public static volatile double SHOOTING_SECONDS = 3.0;
}
