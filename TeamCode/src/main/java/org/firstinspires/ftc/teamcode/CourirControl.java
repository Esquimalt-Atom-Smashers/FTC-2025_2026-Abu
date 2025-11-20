//package org.firstinspires.ftc.teamcode;
//
//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
//import org.firstinspires.ftc.robotcore.external.JavaUtil;
//import java.util.ArrayList;
//import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//
//import java.util.Arrays;
//import java.util.List;
//
//public class CourirControl extends BlocksOpModeCompanion {
//    public static double driveP = 0.03;
//    public static double strafeP = -0.03;
//    public static double turnP = 0.2;
//
//    public static double POSITIONAL_TOLARENCE = 1.0;
//    public static double ANGULAR_TOLARENCE = 1.0;
//
//    @ExportToBlocks(
//            comment = "Courir means 'to run' in French",
//            tooltip = "a robotCentric drive",
//            parameterLabels = {"drive", "strafe", "turn"}
//    )
//
//    public static double[] robotCentricDrive(double rDrive, double rStrafe, double turn) {
//        double frontLeftPower = rDrive + rStrafe + turn;
//        double frontRightPower = rDrive - rStrafe - turn;
//        double rearLeftPower = rDrive - rStrafe + turn;
//        double rearRightPower = rDrive + rStrafe - turn;
//
//        return new double[] {frontLeftPower, frontRightPower, rearLeftPower, rearRightPower};
//    }
//
//    @ExportToBlocks(
//            comment = "Courir means 'to run' in French",
//            tooltip = "a fieldCentric drive that only takes 0 as 'forward'",
//            parameterLabels = {"currentPose2D", "drive", "strafe", "turn"}
//    )
//    public static double[] fieldCentricDrive(Pose2D currentPose, double fDrive, double fStrafe, double turn) {
//        double botHeading = currentPose.getHeading(AngleUnit.RADIANS);
//        double rDrive = fDrive * Math.cos(botHeading) - fStrafe * Math.sin(botHeading);
//        double rStrafe = fDrive * Math.sin(botHeading) + fStrafe * Math.cos(botHeading);
//
//        return robotCentricDrive(rDrive, rStrafe, turn);
//    }
//
//    // @ExportToBlocks(
//    //         comment = "DO NOT TOUCH without consulting",
//    //         tooltip = "DO NOT TOUCH without consulting",
//    //         parameterLabels = {"driveP", "strafeP", "turnP"}
//    // )
//    // public static void setDriveP(double driveP, double strafeP, double turnP) {
//    //     CourirControl.driveP = driveP;
//    //     CourirControl.strafeP = strafeP;
//    //     CourirControl.turnP = turnP;
//    // }
//
//    @ExportToBlocks(
//            comment = "drive to a coordinate on a field",
//            tooltip = "drive to a coordinate on a field",
//            parameterLabels = {"targetPose2D", "currentPose2D", "maxPower", "minPower"}
//    )
//    public static double[] driveToPose2D(Pose2D targetPose, Pose2D currentPose, double maxPower, double minPower) {
//        double tX = targetPose.getX(DistanceUnit.INCH);
//        double cX = currentPose.getX(DistanceUnit.INCH);
//
//        double xError = tX - cX;
//        double drivePower = driveP * xError;
//
//        double tY = targetPose.getY(DistanceUnit.INCH);
//        double cY = currentPose.getY(DistanceUnit.INCH);
//        double yError = tY - cY;
//
//        double strafePower = strafeP * yError;
//
//        double tHeading = targetPose.getHeading(AngleUnit.RADIANS);
//        double cHeading = currentPose.getHeading(AngleUnit.RADIANS);
//        double headingError = AngleUnit.normalizeRadians(tHeading - cHeading);
//
//        double turnPower = turnP * headingError;
//
//        drivePower = applyMaxMinPower(drivePower, maxPower, minPower);
//        strafePower = applyMaxMinPower(strafePower, maxPower, minPower);
//        turnPower = applyMaxMinPower(turnPower, maxPower, minPower);
//
//        double denominator = Math.max(Math.abs(drivePower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
//
//        return fieldCentricDrive(currentPose, drivePower / denominator, strafePower / denominator, turnPower / denominator);
//    }
//
//    @ExportToBlocks(
//            comment = "determine if you are at a pose",
//            tooltip = "determine if you are at a pose",
//            parameterLabels = {"targetPose2D", "currentPose2D"}
//    )
//    public static boolean isNotOnPose(Pose2D targetPose, Pose2D currentPose) {
//        double tX = targetPose.getX(DistanceUnit.INCH);
//        double cX = currentPose.getX(DistanceUnit.INCH);
//        telemetry.addData("errorX", Math.abs(tX - cX));
//        boolean isDriveOnPose = Math.abs(tX - cX) <= POSITIONAL_TOLARENCE;
//
//        double tY = targetPose.getY(DistanceUnit.INCH);
//        double cY = currentPose.getY(DistanceUnit.INCH);
//        boolean isStrafeOnPose = Math.abs(tY - cY) <= POSITIONAL_TOLARENCE;
//
//
//        double tHeading = targetPose.getHeading(AngleUnit.DEGREES);
//        double cHeading = currentPose.getHeading(AngleUnit.DEGREES);
//        double headingError = AngleUnit.normalizeDegrees(tHeading - cHeading);
//        boolean isHeadingOnPose = Math.toDegrees(headingError) <= ANGULAR_TOLARENCE;
//        // telemetry.addData("isDriveOnPOse", isDriveOnPose);
//        // telemetry.addData("isStrafeOnPOse", isStrafeOnPose);
//        // telemetry.addData("isHeadingOnPOse", isHeadingOnPose);
//
//        return !(isDriveOnPose && isStrafeOnPose && isHeadingOnPose);
//    }
//
//    private static double applyMaxMinPower(double val, double maxPower, double minPower) {
//        if (Math.abs(val) < minPower && Math.abs(val) > 0.005) {
//            return Math.signum(val) * minPower;
//        }
//        if (Math.abs(val) > maxPower) {
//            return Math.signum(val) * maxPower;
//        }
//        return val;
//    }
//}
