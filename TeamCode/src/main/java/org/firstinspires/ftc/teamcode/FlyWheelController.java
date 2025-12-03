package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

@Config
public class FlyWheelController extends BlocksOpModeCompanion {
    public static class Params {
        public double TOLERANCE = 28;
        public double kS = 0;
        public double kV = 0.000475;
        public double kA = 0;

        public double P = 0.005;
        public double I = 0.001;
        public double D = 0;

        public double CLOSE_SHOOTING_ANGLE = 0.0;
        public double FAR_SHOOTING_ANGLE = 1.0;

        public double MAX_FLYWHEEL_POWER = 1.0;
    }
    private static final double TICKS_PER_ROTATION = 28;
    private static PIDController flyWheelController;
    public static Params PARAMS = new Params();

    @ExportToBlocks(
            comment = "",
            tooltip = "initializeFlywheelController",
            parameterLabels = {}
    )

    public static void initializeFlywheelSubsystem() {
        flyWheelController = new PIDController(PARAMS.P, PARAMS.I, PARAMS.D);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "rpmToTps",
            parameterLabels = {"rpm"}
    )
    public static double rpmToTps(double rpm) {
        return rpm * TICKS_PER_ROTATION / 60;
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "rpmToTps",
            parameterLabels = {"tps"}
    )
    public static double getFlywheelRPM(double tps) {
        return -tps / TICKS_PER_ROTATION * 60;
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "flyWheelCustomPID",
            parameterLabels = {"targetVelocity", "currentVelocity"}
    )
    public static double flywheelCustomPID(double targetVelocity, double currentVelocity) {

        double pid = Range.clip(flyWheelController.calculate(currentVelocity, targetVelocity), -PARAMS.MAX_FLYWHEEL_POWER, PARAMS.MAX_FLYWHEEL_POWER);
        return pid;
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "flyWheelFeedForward",
            parameterLabels = {"targetVelocity", "currentVelocity"}
    )
    public static double flywheelFeedForward(double targetVelocity, double currentVelocity) {
        double feedForward = PARAMS.kS * Math.signum(targetVelocity) + PARAMS.kV * targetVelocity + PARAMS.kA * (currentVelocity - targetVelocity);
        return feedForward;
    }
}
