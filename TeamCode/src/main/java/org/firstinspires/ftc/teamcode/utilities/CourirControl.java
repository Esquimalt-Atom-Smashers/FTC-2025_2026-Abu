package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class CourirControl extends BlocksOpModeCompanion {
    public static double driveP = 0.0;
    public static double driveI = 0.0;
    public static double driveD = 0.0;

    public static double strafeP = 0.0;
    public static double strafeI = 0.0;
    public static double strafeD = 0.0;

    public static double turnP = 0.0;
    public static double turnI = 0.0;
    public static double turnD = 0.0;

    @ExportToBlocks(
            comment = "Courir means 'to run' in French",
            tooltip = "a robotCentric drive",
            parameterLabels = {"drive", "strafe", "turn"}
    )

    public static double[] robotCentricDrive(double rDrive, double rStrafe, double turn) {
        double frontLeftPower = rDrive - rStrafe - turn;
        double frontRightPower = rDrive + rStrafe + turn;
        double rearLeftPower = rDrive + rStrafe + turn;
        double rearRightPower = rDrive - rStrafe - turn;

        return new double[] {frontLeftPower, frontRightPower, rearLeftPower, rearRightPower};
    }

    @ExportToBlocks(
            comment = "Courir means 'to run' in French",
            tooltip = "a fieldCentric drive that only takes 0 as 'forward'",
            parameterLabels = {"currentPose2D", "drive", "strafe", "turn"}
    )
    public static double[] fieldCentricDrive(Pose2D currentPose, double fDrive, double fStrafe, double turn) {
        double botHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double rDrive = fStrafe * Math.cos(botHeading) - fDrive * Math.sin(botHeading);
        double rStrafe = fStrafe * Math.sin(botHeading) + fDrive * Math.cos(botHeading);

        return robotCentricDrive(rDrive, rStrafe, turn);
    }

    @ExportToBlocks(
            comment = "DO NOT TOUCH without consulting",
            tooltip = "DO NOT TOUCH without consulting",
            parameterLabels = {"P", "I", "D"}
    )
    public static void setDrivePID(double P, double I, double D) {
        driveP = P;
        driveI = I;
        driveD = D;
    }

    @ExportToBlocks(
            comment = "DO NOT TOUCH without consulting",
            tooltip = "DO NOT TOUCH without consulting",
            parameterLabels = {"P", "I", "D"}
    )
    public static void setStrafePID(double P, double I, double D) {
        strafeP = P;
        strafeI = I;
        strafeD = D;
    }

    @ExportToBlocks(
            comment = "DO NOT TOUCH without consulting",
            tooltip = "DO NOT TOUCH without consulting",
            parameterLabels = {"P", "I", "D"}
    )
    public static void setTurnPID(double P, double I, double D) {
        turnP = P;
        turnI = I;
        turnD = D;
    }

    @ExportToBlocks(
            comment = "drive to a coordinate on a field",
            tooltip = "drive to a coordinate on a field",
            parameterLabels = {"targetPose2D", "currentPose2D"}
    )
    public static double[] driveToPose2D(Pose2D targetPose, Pose2D currentPose) {
        double tX = targetPose.getX(DistanceUnit.INCH);
        double cX = currentPose.getX(DistanceUnit.INCH);

        double drivePower = new PIDController(driveP, driveI, driveD).calculate(cX, tX);

        double tY = targetPose.getY(DistanceUnit.INCH);
        double cY = currentPose.getY(DistanceUnit.INCH);

        double strafePower = new PIDController(strafeP, strafeI, strafeD).calculate(cY, tY);

        double tHeading = targetPose.getHeading(AngleUnit.RADIANS);
        double cHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double headingError = AngleUnit.normalizeRadians(tHeading - cHeading);

        double turnPower = new PIDController(turnP, turnI, turnD).calculate(0, headingError);

        return fieldCentricDrive(currentPose, drivePower, strafePower, turnPower);
    }
}
