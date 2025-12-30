package org.firstinspires.ftc.teamcode.subsystems;

/**
 * IntakeTransferSubsystem
 *
 * Handles intaking balls, transferring them, ejecting,
 * and feeding the shooter.
 *
 * This class currently contains only structure and documentation.
 * Hardware logic should be added later.
 */
public class IntakeTransferSubsystem implements SubsystemBase{

    /**
     * Returns the current state of the subsystem.
     *
     * @return The current subsystem state
     */
    public IntakeTransferSubsystem() {

    }

    /**
     * Represents the color of a ball detected or handled
     * by the intake/transfer system.
     */
    public enum BallColour {
        PURPLE,
        GREEN,
        BOTH
    }

    /**
     * Represents the direction the intake should run.
     */
    public enum Direction {
        LEFT,
        RIGHT
    }

    public enum State{

    }

    /**
     * Runs the intake to collect or move balls.
     *
     * @param ballColour The color of ball to intake
     * @param numberOfBalls The number of balls to intake
     * @param direction The direction to run the intake
     */
    public void intake(BallColour ballColour, int numberOfBalls, Direction direction) {
        // TODO: Implement intake logic
    }

    /**
     * Stops all intake and transfer motion.
     */
    public void stop() {
        // TODO: Stop motors/servos
    }

    /**
     * Ejects a specified number of balls of a given color.
     *
     * @param numberOfBalls The number of balls to eject
     * @param ballColour The color of balls to eject
     */
    public void eject(int numberOfBalls, BallColour ballColour) {
        // TODO: Implement eject logic
    }

    /**
     * Feeds a ball of a specific color into the shooter.
     *
     * @param ballColour The color of ball to feed
     */
    public void feedShooter(BallColour ballColour) {
        // TODO: Implement feeding logic
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

