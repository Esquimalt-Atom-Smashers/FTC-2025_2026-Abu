package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class PIDController extends BlocksOpModeCompanion {
     @ExportToBlocks (
         comment = "Magical number box that help a number meet another number",
         tooltip = "enter and tune PID value",
         parameterLabels = {"targetValue", "currentValue", "P", "I", "D"}
     )

    public static double calculate(double targetValue, double currentValue, double P, double I, double D) {
         com.arcrobotics.ftclib.controller.PIDController pidController = new com.arcrobotics.ftclib.controller.PIDController(P, I, D);
         return pidController.calculate(targetValue, currentValue);
     }

}
