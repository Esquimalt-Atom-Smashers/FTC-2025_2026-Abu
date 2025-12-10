package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DriveSubsystem extends SubsystemBase{
    public OpMode opMode;

    private boolean isFieldCentric;
    private double speedMultiplier = 1.0;

    private MecanumDrive mecanumDrive;
    private Pose2d currentPose;
    public double driveHeadingError = 0.0;

    public DriveSubsystem(OpMode opMode, Pose2d startingPose) {
        this.opMode = opMode;
        mecanumDrive = new MecanumDrive(opMode.hardwareMap, startingPose);
        getMecanumDrive().localizer.setPose(startingPose);
        isFieldCentric = true;

        setDriveHeadingError();
    }

    public void drive(double drive, double strafe, double turn) {
        if (isFieldCentric) {
            double botHeading = getHeading();
            double rotX = strafe * Math.cos(botHeading) - drive * Math.sin(botHeading);
            double rotY = strafe * Math.sin(botHeading) + drive * Math.cos(botHeading);

            mecanumDrive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(rotY * speedMultiplier, rotX * speedMultiplier), turn * speedMultiplier
                    ));
        } else {
            mecanumDrive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(drive * speedMultiplier, strafe * speedMultiplier), turn * speedMultiplier
                    ));
        }
    }

    public double getHeading() {
        //get radian
        double headingRadian = -(driveHeadingError - mecanumDrive.localizer.getPose().heading.toDouble());
        if (Math.toDegrees(headingRadian) >= 180) {
            headingRadian -= Math.toRadians(360);
        } else if (Math.toDegrees(headingRadian) < -180) {
            headingRadian += Math.toRadians(360);
        }
        return headingRadian;
    }

    public void switchFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void changeSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = Range.clip(speedMultiplier, 0, 1);
    }

    public void setDriveHeadingError() {
        driveHeadingError = mecanumDrive.localizer.getPose().heading.toDouble();
    }

    public void setDriveHeadingErrorTo(double fieldForwardRadians) {
        driveHeadingError = fieldForwardRadians;
    }

    public Pose2d getCurrentPos() {
        mecanumDrive.updatePoseEstimate();
        currentPose = mecanumDrive.localizer.getPose();
        return currentPose;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    @Override
    public void periodic() {
        opMode.telemetry.addData("fieldCentric heading", Math.toDegrees(getHeading()));
//        opMode.telemetry.addData("is fieldCentric", isFieldCentric);
        opMode.telemetry.addData("current PoseX", getCurrentPos().position.x);
        opMode.telemetry.addData("current PoseY", getCurrentPos().position.y);
        opMode.telemetry.addData("current heading", Math.toDegrees(getCurrentPos().heading.toDouble()));
    }
}
