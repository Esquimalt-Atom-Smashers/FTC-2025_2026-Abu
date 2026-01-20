package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

@Config
public class Property {
    //GENERAL SETTINGS
    public static volatile double TELEMETRY_UPDATE_TIME = 1.0;

    // GOAL POSITIONS
    public static volatile double RED_GOAL_X = -72;
    public static volatile double RED_GOAL_Y = 55;
    public static volatile double RED_GOAL_HEADING = 0;

    public static volatile double BLUE_GOAL_X = -72;
    public static volatile double BLUE_GOAL_Y = -55;
    public static volatile double BLUE_GOAL_HEADING = 0;

    // AUTO STARTING POSITION
    public static volatile double RED_FAR_X = 65;
    public static volatile double RED_FAR_Y = 16;
    public static volatile double RED_FAR_HEADING = 180;

    public static volatile double RED_CLOSE_X = -52.5;
    public static volatile double RED_CLOSE_Y = 54;
    public static volatile double RED_CLOSE_HEADING = 36.5;

    public static volatile double BLUE_FAR_X = 72 - 7;
    public static volatile double BLUE_FAR_Y = -16;
    public static volatile double BLUE_FAR_HEADING = 180;

    public static volatile double BLUE_CLOSE_X = -52.5;
    public static volatile double BLUE_CLOSE_Y = -54;
    public static volatile double BLUE_CLOSE_HEADING = 143.65;

//    RED SHOOTING POSITIONS
    public static volatile double RED_FAR_SHOOT_X = 55;
    public static volatile double RED_FAR_SHOOT_Y = 17;
    public static volatile double RED_FAR_SHOOT_HEADING = 74;

    public static volatile double RED_CLOSE_SHOOT_X = -23;
    public static volatile double RED_CLOSE_SHOOT_Y = 24;
    public static volatile double RED_CLOSE_SHOOT_HEADING = 40;

//    BLUE SHOOTING POSITIONS
    public static volatile double BLUE_FAR_SHOOT_X = 55;
    public static volatile double BLUE_FAR_SHOOT_Y = -17;
    public static volatile double BLUE_FAR_SHOOT_HEADING = 110;

    public static volatile double BLUE_CLOSE_SHOOT_X = -23;
    public static volatile double BLUE_CLOSE_SHOOT_Y = -24;
    public static volatile double BLUE_CLOSE_SHOOT_HEADING = 130;

//    RED INTAKE POSITIONS
    public static volatile double RED_FIRST_INTAKE_P1_X = -18;
    public static volatile double RED_FIRST_INTAKE_P1_Y = 40;
    public static volatile double RED_FIRST_INTAKE_P1_HEADING = 90;

    public static volatile double RED_FIRST_INTAKE_P2_X = -18;
    public static volatile double RED_FIRST_INTAKE_P2_Y = 60;
    public static volatile double RED_FIRST_INTAKE_P2_HEADING = 90;

    public static volatile double RED_SECOND_INTAKE_P1_X = 12;
    public static volatile double RED_SECOND_INTAKE_P1_Y = 40;
    public static volatile double RED_SECOND_INTAKE_P1_HEADING = 90;

    public static volatile double RED_SECOND_INTAKE_P2_X = 12;
    public static volatile double RED_SECOND_INTAKE_P2_Y = 65;
    public static volatile double RED_SECOND_INTAKE_P2_HEADING = 90;

    public static volatile double RED_THIRD_INTAKE_P1_X = 38;
    public static volatile double RED_THIRD_INTAKE_P1_Y = 30;
    public static volatile double RED_THIRD_INTAKE_P1_HEADING = 90;

    public static volatile double RED_THIRD_INTAKE_P2_X = 38;
    public static volatile double RED_THIRD_INTAKE_P2_Y = 55;
    public static volatile double RED_THIRD_INTAKE_P2_HEADING = 90;

    public static volatile double RED_GATE_INTAKE_P1_X = 8;
    public static volatile double RED_GATE_INTAKE_P1_Y = 62;
    public static volatile double RED_GATE_INTAKE_P1_HEADING = 110;

    public static volatile double RED_GATE_INTAKE_P2_X = 12;
    public static volatile double RED_GATE_INTAKE_P2_Y = 63;
    public static volatile double RED_GATE_INTAKE_P2_HEADING = 110;

    public static volatile double GATE_INTAKE_DELAY_SECOND = 2.0;

    public static volatile double RED_LOAD_INTAKE_P1_X = 48;
    public static volatile double RED_LOAD_INTAKE_P1_Y = 65;
    public static volatile double RED_LOAD_INTAKE_P1_HEADING = 22.5;

    public static volatile double RED_LOAD_INTAKE_P2_X = 70;
    public static volatile double RED_LOAD_INTAKE_P2_Y = 65;
    public static volatile double RED_LOAD_INTAKE_P2_HEADING = 0;

//    BLUE INTAKE POSITIONS
    public static volatile double BLUE_FIRST_INTAKE_P1_X = -12;
    public static volatile double BLUE_FIRST_INTAKE_P1_Y = -40;
    public static volatile double BLUE_FIRST_INTAKE_P1_HEADING = 270;

    public static volatile double BLUE_FIRST_INTAKE_P2_X = -12;
    public static volatile double BLUE_FIRST_INTAKE_P2_Y = -60;
    public static volatile double BLUE_FIRST_INTAKE_P2_HEADING = 270;

    public static volatile double BLUE_SECOND_INTAKE_P1_X = 12;
    public static volatile double BLUE_SECOND_INTAKE_P1_Y = -40;
    public static volatile double BLUE_SECOND_INTAKE_P1_HEADING = 270;

    public static volatile double BLUE_SECOND_INTAKE_P2_X = 12;
    public static volatile double BLUE_SECOND_INTAKE_P2_Y = -65;
    public static volatile double BLUE_SECOND_INTAKE_P2_HEADING = 270;

    public static volatile double BLUE_THIRD_INTAKE_P1_X = 38;
    public static volatile double BLUE_THIRD_INTAKE_P1_Y = -30;
    public static volatile double BLUE_THIRD_INTAKE_P1_HEADING = 270;

    public static volatile double BLUE_THIRD_INTAKE_P2_X = 38;
    public static volatile double BLUE_THIRD_INTAKE_P2_Y = -55;
    public static volatile double BLUE_THIRD_INTAKE_P2_HEADING = 270;

    public static volatile double BLUE_LOAD_INTAKE_P1_X = 48;
    public static volatile double BLUE_LOAD_INTAKE_P1_Y = -65;
    public static volatile double BLUE_LOAD_INTAKE_P1_HEADING = 0;

    public static volatile double BLUE_LOAD_INTAKE_P2_X = 70;
    public static volatile double BLUE_LOAD_INTAKE_P2_Y = -65;
    public static volatile double BLUE_LOAD_INTAKE_P2_HEADING = 0;

    public static volatile double BLUE_GATE_INTAKE_P1_X = 8;
    public static volatile double BLUE_GATE_INTAKE_P1_Y = -62;
    public static volatile double BLUE_GATE_INTAKE_P1_HEADING = 290;

    public static volatile double BLUE_GATE_INTAKE_P2_X = 12;
    public static volatile double BLUE_GATE_INTAKE_P2_Y = -63;
    public static volatile double BLUE_GATE_INTAKE_P2_HEADING = 290;

    //FLYWHEEL CONTROL
    public static volatile double FAR_SHOOT_RPM = 3450;
    public static volatile double CLOSE_SHOOT_RPM = 3050;

    public static volatile double TOLERANCE = 100;
    public static volatile double kV = 0.000215;
    public static volatile double P = 0.005;

    public static volatile double SHOOTING_SECONDS = 5.0;

    public static volatile double THIRTY_INCH_RPM = 3300;
    public static volatile double FORTY_FIVE_INCH_RPM = 3050;
    public static volatile double SIXTY_INCH_RPM = 3100;
    public static volatile double SEVENTY_FIVE_INCH_RPM = 3100;
    public static volatile double NINETY_INCH_RPM = 3150;
    public static volatile double ONE_HUNDRED_FIVE_INCH_RPM = 3350;
    public static volatile double ONE_HUNDRED_TWENTY_INCH_RPM = 3350;
    public static volatile double ONE_HUNDRED_THIRTY_INCH_RPM = 3400;

    public static void reset() {
        Field[] fields = Property.class.getDeclaredFields();

        for (Field field : fields) {
            try {
                // Only touch static doubles
                if (!Modifier.isStatic(field.getModifiers()) ||
                        field.getType() != double.class) {
                    continue;
                }

                Field defaultField =
                        PropertyDefaults.class.getDeclaredField(field.getName());

                double defaultValue = defaultField.getDouble(null);
                field.setDouble(null, defaultValue);

            } catch (NoSuchFieldException ignored) {
                // Field exists in Property but not in PropertyDefaults → skip
            } catch (IllegalAccessException ignored) {
                // Should never happen if fields are public
            }
        }
    }
}
