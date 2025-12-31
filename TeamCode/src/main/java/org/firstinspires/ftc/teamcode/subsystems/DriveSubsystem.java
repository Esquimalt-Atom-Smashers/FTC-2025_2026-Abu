package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * DriveSubsystem
 *
 * Handles all robot movement, including field-centric and
 * robot-centric driving, pose tracking, and autonomous navigation.
 *
 * This class currently contains only structure and documentation.
 */
public class DriveSubsystem implements SubsystemBase {

    public OpMode opMode;

    private boolean isFieldCentric;
    private double speedMultiplier = 1.0;

    private MecanumDrive mecanumDrive;
    private Pose2d currentPose;
    public double driveHeadingError = 0.0;
    private boolean isTelemetryEnabled = true;

    public enum DriveSubsystemState{
        DISABLED,
        TELEOP_DRIVING,
        TELEOP_AIMBOT,
        AUTO
    }
    public DriveSubsystemState currentState;
    /**
     * Constructor for the DriveSubsystem.
     * <p>
     * Drive motors, IMU, and localization systems
     * should be initialized here later.
     */
    public DriveSubsystem(OpMode opMode, Pose2d startingPose, DriveSubsystemState state) {
        this.opMode = opMode;
        mecanumDrive = new MecanumDrive(opMode.hardwareMap, startingPose);
        getMecanumDrive().localizer.setPose(startingPose);
        isFieldCentric = true;

        setDriveHeadingError();
        currentState = state;
    }

    /**
     * Drives the robot using field-centric control.
     *
     * @param drive  Forward/backward movement input
     * @param strafe Left/right movement input
     * @param turn   Rotational movement input
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {
        double botHeading = getHeading();
        double rotX = strafe * Math.cos(botHeading) - drive * Math.sin(botHeading);
        double rotY = strafe * Math.sin(botHeading) + drive * Math.cos(botHeading);

        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(rotY * speedMultiplier, rotX * speedMultiplier), turn * speedMultiplier
                ));
    }

    /**
     * Drives the robot using robot-centric control.
     *
     * @param drive  Forward/backward movement input
     * @param strafe Left/right movement input
     * @param turn   Rotational movement input
     */
    public void driveRobotCentric(double drive, double strafe, double turn) {
        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(drive * speedMultiplier, strafe * speedMultiplier), turn * speedMultiplier
                ));
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

    /**
     *@return heading for fieldCentric teleOp driving
     */
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
    /**
     * @return The current Pose (position and heading)
     */
    public Pose2d getPose() {
        mecanumDrive.updatePoseEstimate();
        return currentPose = mecanumDrive.localizer.getPose();
    }

    /**
     * Sets the robot's current pose.
     * @param pose The pose to set as the current position
     */
    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

    /**
     * Commands the robot to navigate to a target pose.
     * @param pose The target pose to drive to
     */
    public void goToPose(Pose2d pose) {
        // TODO: Implement path following or PID control, if we want to.
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }
//--------------------Common functions across subsystems--------------------
    /** shall be ran every loop*/
    @Override
    public void periodic() {
        if (currentState == DriveSubsystemState.DISABLED) {
            shutDownSubsystem();
        }
    }

    /**
     * Enables or disables telemetry output for this subsystem.
     * @param enabled True to enable telemetry, false to disable
     */
    @Override
    public void enableSubsystemTelemetry(boolean enabled) {
        isTelemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if (isTelemetryEnabled) {
            Pose2d pose = getPose();
            opMode.telemetry.addData("Pose", "X: %d, Y: %d, H: %d", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            opMode.telemetry.addData("FC Heading", getHeading());
        }
    }

    /**Resets the subsystem to a known safe state.*/
    @Override
    public void resetSubsystem() {

    }

    /**
     * Safely shuts down the subsystem.
     * Motors should stop and resources should be released.
     */
    @Override
    public void shutDownSubsystem() {
        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0), 0
                )
        );
    }

    /**
     * Returns the current state of the subsystem.
     *@return The current subsystem state (subsystem-specific enum)
     */
    @Override
    public DriveSubsystemState getState() {
        return currentState;
    }

    public void setState(DriveSubsystemState targetState) {
        currentState = targetState;
    }

}
