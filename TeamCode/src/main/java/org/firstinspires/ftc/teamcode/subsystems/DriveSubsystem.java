package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * DriveSubsystem
 *
 * Handles all robot movement, including field-centric and
 * robot-centric driving, pose tracking, and autonomous navigation.
 *
 * This class currently contains only structure and documentation.
 */
public class DriveSubsystem {

    /**
     * Constructor for the DriveSubsystem.
     *
     * Drive motors, IMU, and localization systems
     * should be initialized here later.
     */
    public DriveSubsystem() {
        // Initialize drive hardware and sensors here
    }

    /**
     * Drives the robot using field-centric control.
     *
     * @param drive Forward/backward movement input
     * @param strafe Left/right movement input
     * @param turn Rotational movement input
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {
        // TODO: Convert field-centric inputs to robot-centric motion
        // TODO: Apply motor power
    }

    /**
     * Drives the robot using robot-centric control.
     *
     * @param drive Forward/backward movement input
     * @param strafe Left/right movement input
     * @param turn Rotational movement input
     */
    public void driveRobotCentric(double drive, double strafe, double turn) {
        // TODO: Apply motor power directly in robot reference frame
    }

    /**
     * Resets the robot's heading to zero.
     *
     * Typically called at the start of a match or autonomous.
     */
    public void resetHeading() {
        // TODO: Reset IMU heading
    }

    /**
     * Returns the robot's current pose.
     *
     * @return The current Pose (position and heading)
     */
    public Pose2d getPose() {
        // TODO: Return current pose from localization
        return null;
    }

    /**
     * Sets the robot's current pose.
     *
     * @param pose The pose to set as the current position
     */
    public void setPose(Pose2d pose) {
        // TODO: Update localization with new pose
    }

    /**
     * Commands the robot to navigate to a target pose.
     *
     * @param pose The target pose to drive to
     */
    public void goToPose(Pose2d pose) {
        // TODO: Implement path following or PID control
    }

//--------------------Common functions across subsystems--------------------
    /**
     * Runs every loop
     */
    public void periodic() {

    }

    /**
     * Enables or disables telemetry output for this subsystem.
     *
     * @param enabled True to enable telemetry, false to disable
     */
    public void enableSubsystemTelemetry(boolean enabled) {

    }

    /**
     * Resets the subsystem to a known safe state.
     */
    public void resetSubsystem(){

    }

    /**
     * Safely shuts down the subsystem.
     *
     * Motors should stop and resources should be released.
     */
    public void shutDownSubsystem() {

    }

    public Enum<?> getState() {
        return null;
    }
}
