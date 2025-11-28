package org.firstinspires.ftc.teamcode;

public class PIDController{
    private com.arcrobotics.ftclib.controller.PIDController controller;

    public PIDController(double P, double I, double D) {
         controller = new com.arcrobotics.ftclib.controller.PIDController(P, I, D);
    }
    public double calculate(double targetValue, double currentValue) {
         return controller.calculate(targetValue, currentValue);
     }

}
