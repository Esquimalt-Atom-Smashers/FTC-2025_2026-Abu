package org.firstinspires.ftc.teamcode.subsystems;
/**
 * ShooterSubsystem
 *
 * Controls the shooter mechanism, including flywheel speed
 * and hood angle adjustment.
 *
 * This class currently contains only structure and documentation.
 */
public class ShooterSubsystem implements SubsystemBase{

    /**
     * Represents the alliance color.
     * Used for automatically selecting shooting behavior.
     */
    public enum Alliance {
        RED,
        BLUE
    }

    /**
     * Constructor for the ShooterSubsystem.
     *
     * Shooter motors, hood servos, and sensors
     * should be initialized here later.
     */
    public ShooterSubsystem() {
        // Initialize shooter hardware here
    }

    /**
     * Shoots using a specific hood angle and flywheel RPM.
     *
     * @param hoodAngle The desired hood angle (units defined by implementation)
     * @param rpm The target flywheel speed in RPM
     */
    public void shoot(double hoodAngle, double rpm) {
        // TODO: Set hood angle
        // TODO: Set shooter motor RPM
    }

    /**
     * Shoots using preconfigured settings based on alliance color.
     *
     * @param alliance The current alliance (RED or BLUE)
     */
    public void shoot(Alliance alliance) {
        // TODO: Select hood angle and RPM based on alliance
        // TODO: Call shoot(angle, rpm)
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
