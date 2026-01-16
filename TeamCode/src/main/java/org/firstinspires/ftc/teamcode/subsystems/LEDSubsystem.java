package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

public class LEDSubsystem implements SubsystemBase {

    public enum LEDState {
        RED_NORMAL,
        BLUE_NORMAL,
        RED_AIMING,
        BLUE_AIMING,
        RED_AIMED,
        BLUE_AIMED,
        RED_PARKING,
        BLUE_PARKING,
        TEAM_COLOUR
    }
    private final OpMode opMode;
    private final RevBlinkinLedDriver blinkin;
    private LEDState currentState;
    private boolean telemetryEnabled = true;

    private final RobotContainer.Alliance alliance;

    public LEDSubsystem(OpMode opMode, RobotContainer.Alliance alliance) {
        this.opMode = opMode;
        this.blinkin = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        this.alliance = alliance;
    }

    @Override
    public void periodic() {
        // LEDs are state-based. If you're updating them every loop,
        // you're wasting cycles.
    }

    public void setState(LEDState newState) {
        if (newState == currentState) return;
        currentState = newState;
        applyState(newState);
    }

    private void applyState(LEDState state) {
        switch (state) {
            case RED_NORMAL:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case BLUE_NORMAL:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            case RED_AIMING:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
                break;
            case BLUE_AIMING:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                break;
            case RED_AIMED:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                break;
            case BLUE_AIMED:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                break;
            case RED_PARKING:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                break;
            case BLUE_PARKING:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                break;
            case TEAM_COLOUR:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
        }
    }

    public void normalLight() {
        if (alliance == RobotContainer.Alliance.RED) {
            applyState(LEDState.RED_NORMAL);
        } else {
            applyState(LEDState.BLUE_NORMAL);
        }
    }
    public void aimingLight() {
        if (alliance == RobotContainer.Alliance.RED) {
            applyState(LEDState.RED_AIMING);
        } else {
            applyState(LEDState.BLUE_AIMING);
        }
    }

    public void aimedLight() {
        if (alliance == RobotContainer.Alliance.RED) {
            applyState(LEDState.RED_AIMED);
        } else {
            applyState(LEDState.BLUE_AIMED);
        }
    }

    public void parkingLight() {
        if (alliance == RobotContainer.Alliance.RED) {
            applyState(LEDState.RED_PARKING);
        } else {
            applyState(LEDState.BLUE_PARKING);
        }
    }

    public void teamColourLight() {
        applyState(LEDState.TEAM_COLOUR);
    }

    @Override
    public void enableSubsystemTelemetry(boolean enabled) {
        telemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if (!telemetryEnabled) return;
        // Add telemetry in RobotContainer; subsystem shouldn't own Telemetry
    }

    @Override
    public void resetSubsystem() {
        setState(LEDState.TEAM_COLOUR);
    }

    @Override
    public void shutDownSubsystem() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    @Override
    public Enum<?> getState() {
        return currentState;
    }
}
