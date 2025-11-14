package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class RobotContainer extends BlocksOpModeCompanion {
    public static double x = 0.0;
    public static double y = 0.0;
    public static double heading = 0.0;

    public static boolean isValid = false;
    public static boolean received = false;

    @ExportToBlocks (
        comment = "Store bot pos from auto",
        tooltip = "Pass x, y, heading for TeleOp",
        parameterLabels = {"x", "y", "heading"}
    )
    public static void storePos(double x, double y, double heading){
        RobotContainer.x = x;
        RobotContainer.y = y;
        RobotContainer.heading = heading;

        isValid = true;
        received = false;
    }

    @ExportToBlocks (
            comment = "",
            tooltip = "Give you bot X pos"
    )
    public static double getX() { return x; }

    @ExportToBlocks (
            comment = "",
            tooltip = "Give you bot Y pos"
    )
    public static double getY() { return y; }

    @ExportToBlocks (
            comment = "",
            tooltip = "Give you bot Heading"
    )
    public static double getHeading() { return heading; }

    @ExportToBlocks (
            comment = "",
            tooltip = "Makes sure the data is valid"
    )
    public static boolean hasData() {
        return isValid && !received;
    }

    @ExportToBlocks (
            comment = "",
            tooltip = "Mark the data once you have seen it"
    )
    public static void markReceived() {
        received = true;
    }

    @ExportToBlocks (
            comment = "",
            tooltip = "Delete not useful data"
    )
    public static void clear() {
        x = 0;
        y = 0;
        heading = 0;
        isValid = false;
        received = false;
    }
}
