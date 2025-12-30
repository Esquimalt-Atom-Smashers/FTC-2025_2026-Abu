package org.firstinspires.ftc.teamcode.subsystems;

public interface SubsystemBase {
    /**
     * Subsystem
     *
     * Common interface implemented by all robot subsystems.
     * Provides lifecycle control, telemetry control, and state access.
     */
    public interface Subsystem {
        /**
         * Runs every loop
         */
        void periodic();

        /**
         * Enables or disables telemetry output for this subsystem.
         *
         * @param enabled True to enable telemetry, false to disable
         */
        void enableSubsystemTelemetry(boolean enabled);

        /**
         * Resets the subsystem to a known safe state.
         */
        void resetSubsystem();

        /**
         * Safely shuts down the subsystem.
         *
         * Motors should stop and resources should be released.
         */
        void shutDownSubsystem();

        /**
         * Returns the current state of the subsystem.
         *
         * @return The current subsystem state (subsystem-specific enum)
         */
        Enum<?> getState();

    }

}
