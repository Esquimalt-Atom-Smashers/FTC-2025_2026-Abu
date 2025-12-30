package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * VisionSubsystem
 *
 * Handles vision processing and targeting using a vision system
 * (e.g., Limelight, OpenCV, AprilTags).
 *
 * This class currently contains only structure and documentation.
 */
public class VisionSubsystem {

    /**
     * Represents the alliance color.
     * Used for selecting alliance-specific targets.
     */
    public enum Alliance {
        RED,
        BLUE
    }

    /**
     * Represents possible detected field patterns.
     */
    public enum Pattern {
        PATTERN1,
        PATTERN2,
        PATTERN3
    }

    /**
     * Constructor for the VisionSubsystem.
     *
     * Vision hardware and pipelines should be
     * initialized here later.
     */
    public VisionSubsystem() {
        // Initialize vision hardware and pipelines here
    }

    /**
     * Returns the robot's position estimated by the Limelight.
     *
     * @param currentHeading The robot's current heading (from IMU)
     * @return The estimated pose from vision
     */
    public Pose2d getLimelightPos(double currentHeading) {
        // TODO: Read Limelight data and compute pose
        return null;
    }

    /**
     * Returns the heading the robot should turn to
     * in order to face the target for the given alliance.
     *
     * @param alliance The current alliance
     * @return Target heading in radians or degrees (define later)
     */
    public double getTargetHeading(Alliance alliance) {
        // TODO: Calculate target heading from vision data
        return 0.0;
    }

    /**
     * Returns the detected field pattern or target configuration.
     *
     * @return Identifier for the detected pattern
     */
    public Pattern getPattern() {
        // TODO: Determine pattern from vision pipeline
        return null;
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
