package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CourirControl;

public class DriveSubsystem extends BlocksOpModeCompanion {
    private static DcMotor frontLeftMotor;
    private static DcMotor frontRightMotor;
    private static DcMotor rearRightMotor;
    private static DcMotor rearLeftMotor;
    private static SparkFunOTOS sensor_otos;
    private static SparkFunOTOS.Pose2D myPose;
    private static double fieldForward;

    @ExportToBlocks(
            comment = "Initialize drive subsystem",
            tooltip = "a fieldCentric drive that only takes 0 as 'forward'",
            parameterLabels = {"currentPose2D"}
    )
    public static void initializeDriveSubsystem(Pose2D currentPose2D) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rearRightMotor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rearLeftMotor");
        sensor_otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        SparkFunOTOS.Pose2D otosOffset;

        sensor_otos.setAngularUnit(AngleUnit.DEGREES);
        sensor_otos.setLinearUnit(DistanceUnit.INCH);
        otosOffset = new SparkFunOTOS.Pose2D(0.3244, -5.3406, AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 270));
        sensor_otos.setOffset(otosOffset);

        double x = currentPose2D.getX(DistanceUnit.INCH);
        double y = currentPose2D.getY(DistanceUnit.INCH);
        double heading = currentPose2D.getHeading(AngleUnit.RADIANS);

        myPose = new SparkFunOTOS.Pose2D(x, y, heading);
        sensor_otos.setPosition(myPose);

        fieldForward = heading;
    }

    @ExportToBlocks(
            comment = "set motor power",
            tooltip = "set motor power",
            parameterLabels = {"fL", "fR", "rL", "rR"}
    )
    public static void setMotorPower(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearRightMotor.setPower(rearLeftPower);
        rearLeftMotor.setPower(rearRightPower);

        myPose = sensor_otos.getPosition();
    }

    @ExportToBlocks(
            comment = "robot centric drive",
            tooltip = "a robot centric drive",
            parameterLabels = {"drive", "strafe", "turn"}
    )
    public static void robotCentricDrive(double drive, double strafe, double turn) {
        double[] powerList = CourirControl.robotCentricDrive(drive, strafe, turn);
        setMotorPower(powerList[0], powerList[1], powerList[2], powerList[3]);
    }

    @ExportToBlocks(
            comment = "field centric drive",
            tooltip = "a field centric drive",
            parameterLabels = {"drive", "strafe", "turn"}
    )
    public static void fieldCentricDrive(double drive, double strafe, double turn) {
        // double fieldCentricHeading = fieldForward - getCurrentPose2D().getHeading(AngleUnit.RADIANS);
        // double[] powerList = CourirControl.fieldCentricDrive(
        //     new Pose2D(DistanceUnit.INCH, myPose.x, myPose.y, AngleUnit.RADIANS, fieldCentricHeading), drive, strafe, turn);
        double[] powerList = CourirControl.fieldCentricDrive(getCurrentPose2D(), drive, strafe, turn);
        setMotorPower(powerList[0], powerList[1], powerList[2], powerList[3]);
    }

    @ExportToBlocks(
            comment = "get current Pose2D",
            tooltip = "get current Pose2D",
            parameterLabels = {}
    )
    public static Pose2D getCurrentPose2D() {
        myPose = sensor_otos.getPosition();
        return new Pose2D(DistanceUnit.INCH, myPose.x, myPose.y, AngleUnit.RADIANS, myPose.h);
    }
}