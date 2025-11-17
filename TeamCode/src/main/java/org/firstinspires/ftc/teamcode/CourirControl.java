package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;

public class CourirControl extends BlocksOpModeCompanion {
    public static double driveP = 0.0;
    public static double strafeP = 0.0;
    public static double turnP = 0.0;

    @ExportToBlocks(
            comment = "Courir means 'to run' in French",
            tooltip = "a robotCentric drive",
            parameterLabels = {"drive", "strafe", "turn"}
    )

    public static List<Double> robotCentricDrive(double rDrive, double rStrafe, double turn) {
        double frontLeftPower = rDrive - rStrafe - turn;
        double frontRightPower = rDrive + rStrafe + turn;
        double rearLeftPower = rDrive + rStrafe + turn;
        double rearRightPower = rDrive - rStrafe - turn;

        return Arrays.asList(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    @ExportToBlocks(
            comment = "Courir means 'to run' in French",
            tooltip = "a fieldCentric drive that only takes 0 as 'forward'",
            parameterLabels = {"currentPose2D", "drive", "strafe", "turn"}
    )
    public static List<Double> fieldCentricDrive(Pose2D currentPose, double fDrive, double fStrafe, double turn) {
        double botHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double rDrive = fStrafe * Math.cos(botHeading) - fDrive * Math.sin(botHeading);
        double rStrafe = fStrafe * Math.sin(botHeading) + fDrive * Math.cos(botHeading);

        return robotCentricDrive(rDrive, rStrafe, turn);
    }

    @ExportToBlocks(
            comment = "DO NOT TOUCH without consulting",
            tooltip = "DO NOT TOUCH without consulting",
            parameterLabels = {"driveP", "strafeP", "turnP"}
    )
    public static void setDriveP(double driveP, double strafeP, double turnP) {
        CourirControl.driveP = driveP;
        CourirControl.strafeP = strafeP;
        CourirControl.turnP = turnP;
    }

    @ExportToBlocks(
            comment = "drive to a coordinate on a field",
            tooltip = "drive to a coordinate on a field",
            parameterLabels = {"targetPose2D", "currentPose2D", "maxPower", "minPower"}
    )
    public static List<Double> driveToPose2D(Pose2D targetPose, Pose2D currentPose, double maxPower, double minPower) {
        double tX = targetPose.getX(DistanceUnit.INCH);
        double cX = currentPose.getX(DistanceUnit.INCH);

        double xError = tX - cX;
        double drivePower = driveP * xError;

        double tY = targetPose.getY(DistanceUnit.INCH);
        double cY = currentPose.getY(DistanceUnit.INCH);
        double yError = tY - cY;

        double strafePower = strafeP * yError;

        double tHeading = targetPose.getHeading(AngleUnit.RADIANS);
        double cHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double headingError = AngleUnit.normalizeRadians(tHeading - cHeading);

        double turnPower = turnP * headingError;

        drivePower = applyMaxMinPower(drivePower, maxPower, minPower);
        strafePower = applyMaxMinPower(strafePower, maxPower, minPower);
        turnPower = applyMaxMinPower(turnPower, maxPower, minPower);

        return fieldCentricDrive(currentPose, drivePower, strafePower, turnPower);
    }

    private static double applyMaxMinPower(double val, double maxPower, double minPower) {
        if (Math.abs(val) < minPower && Math.abs(val) > 0.005) {
            return Math.signum(val) * minPower;
        }
        if (Math.abs(val) > maxPower) {
            return Math.signum(val) * maxPower;
        }
        return val;
    }
}
