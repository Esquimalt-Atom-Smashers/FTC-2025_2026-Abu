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
    public double driveHeadingError = 0.0;
    private final double ANGULAR_TOLERANCE = 1.0;//degrees

    private LimelightSybsystem limelightSybsystem;
    private final Pose2d RED_GOAL_POSE = new Pose2d(-60,60,0);
    private final Pose2d BLUE_GOAL_POSE = new Pose2d(-60,-60,0);

    public DriveSubsystem(OpMode opMode, Pose2d startingPose, LimelightSybsystem limelightSybsystem) {
        this.opMode = opMode;
        mecanumDrive = new MecanumDrive(opMode.hardwareMap, startingPose);
        this.limelightSybsystem = limelightSybsystem;

        isFieldCentric = true;
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
        double headingRadian = mecanumDrive.localizer.getPose().heading.toDouble() + driveHeadingError;
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

    public class ToLaunchHeading implements Action {
        private boolean cancelled = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if( (Math.toDegrees(currentPose.heading.toDouble()) <= ANGULAR_TOLERANCE) || cancelled) {
                return false;
            } else {
                double goalHeadingError = limelightSybsystem.getTy();
                if (goalHeadingError != Double.NaN) {
                    mecanumDrive.localizer.setPose(limelightSybsystem.getBotPose2D(currentPose));

                    return mecanumDrive.actionBuilder(currentPose)
                            .turnTo(Math.toRadians(limelightSybsystem.getTy() + Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble())))
                            .build()
                            .run(telemetryPacket);
                } else {
                    Pose2d goalPos = limelightSybsystem.getIsRedAlliance()? RED_GOAL_POSE: BLUE_GOAL_POSE;
                    double goalPosX = goalPos.position.x;
                    double goalPosY = goalPos.position.y;
                    double robotPosX = getCurrentPos().position.x;
                    double robotPosY = getCurrentPos().position.y;

                    double hyp = Math.sqrt(Math.pow(robotPosX - goalPosX,2) + Math.pow(robotPosY - goalPosY,2));
                    double targetHeading = Math.asin(robotPosX - goalPosX / hyp);

                    opMode.telemetry.addData("goal target heading", targetHeading);
                    return false;
//                    return mecanumDrive.actionBuilder(currentPose)
//                            .turnTo(targetHeading)
//                            .build()
//                            .run(telemetryPacket);
                }

            }
        }

        public void cancelAbruptly() {cancelled = true;}
    }

    public ToLaunchHeading toLaunchHeading() {return new ToLaunchHeading();}

}
