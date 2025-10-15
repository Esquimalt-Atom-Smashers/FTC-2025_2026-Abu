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
    private OpMode opMode;

    private boolean isFieldCentric;
    private double speedMultiplier = 1.0;

    private MecanumDrive mecanumDrive;
    private Pose2d currentPose;

    private final double POSITIONAL_TOLERANCE = 1.0;
    private final double ANGULAR_TOLERANCE = Math.toRadians(1);

    public DriveSubsystem(OpMode opMode, Pose2d startingPose) {
        this.opMode = opMode;
        mecanumDrive = new MecanumDrive(opMode.hardwareMap, startingPose);

        isFieldCentric = true;
    }

    public void drive(double drive, double strafe, double turn) {
        if (isFieldCentric) {
            double botHeading = getHeading();
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            mecanumDrive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(rotX * speedMultiplier, rotY * speedMultiplier), turn * speedMultiplier
                    ));
        } else {
            mecanumDrive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(strafe * speedMultiplier, drive * speedMultiplier), turn * speedMultiplier
                    ));
        }
    }

    private double getHeading() {
        //get radian
        return mecanumDrive.localizer.getPose().heading.toDouble();
    }

    public void switchFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void changeSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = Range.clip(speedMultiplier, 0, 1);
    }

    public Pose2d getCurrentPos() {
        return currentPose;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    @Override
    public void periodic() {
        opMode.telemetry.addData("heading", Math.toDegrees(getHeading()));
        opMode.telemetry.addData("is fieldCentric", isFieldCentric);
        mecanumDrive.updatePoseEstimate();
        currentPose = mecanumDrive.localizer.getPose();
    }

    public class ToLaunchPos implements Action {
        private boolean cancelled = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if((Math.abs(currentPose.position.x) <= POSITIONAL_TOLERANCE && Math.abs(currentPose.position.y) <= POSITIONAL_TOLERANCE && currentPose.heading.toDouble() <= ANGULAR_TOLERANCE) || cancelled) {
                return false;
            } else {
                return mecanumDrive.actionBuilder(currentPose).
                        strafeToSplineHeading(new Vector2d(0, 0), Math.toRadians(0))
                        .build().run(telemetryPacket);
            }
        }

        public void cancelAbruptly() {cancelled = true;}
    }

    public ToLaunchPos toLaunchPos() {return new ToLaunchPos();}

}
