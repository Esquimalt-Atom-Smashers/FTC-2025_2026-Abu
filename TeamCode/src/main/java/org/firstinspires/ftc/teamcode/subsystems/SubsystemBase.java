package org.firstinspires.ftc.teamcode.subsystems;

public interface SubsystemBase {
        boolean isTelemetryEnabled = true;
        /** gets called from robot container every loop*/
        void periodic();

        /**
         * Enables or disables telemetry output for this subsystem. defaults to true
         * @param enabled True to enable telemetry, false to disable
         */
        void enableSubsystemTelemetry(boolean enabled);

        void addSubsystemTelemetry();

        /**
         * Resets the subsystem to a known safe state.
         */
        void resetSubsystem();

        /**Safely shuts down the subsystem. Motors should stop*/
        void shutDownSubsystem();

        /**
         * Returns the current state of the subsystem.
         * @return The current subsystem state (subsystem-specific enum)
         */
        Enum<?> getState();
}
