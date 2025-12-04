package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;

public class ListHelper extends BlocksOpModeCompanion {

    @ExportToBlocks(
            comment = "",
            tooltip = "get double from double[]",
            parameterLabels = {"list", "index"}
    )

    public static double getDoubleFromList(double[] list, int index) {
        return list[index];
    }

}
