package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utilities.Property;

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

    private DcMotor intakeMotor;
    private final String INTAKE_MOTOR_NAME = "intakeMotor";
    private final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private DcMotor feedMotor;
    private final String FEED_MOTOR_NAME = "feedMotor";
    private final DcMotorSimple.Direction FEED_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public enum ServoStates {
        SPINNING(1.0),
        STOPPED(0.0),
        REVERSED(-1.0);

        private double servoPower;
        private ServoStates(double servoPower) {
            this.servoPower = servoPower;
        }
    }

    /**
     * Represents the direction the intake should run.
     */
    public enum Direction {
        LEFT,
        RIGHT
    }

    public enum IntakeTransferState {
        INTAKING,
        EJECTING,
        FEEDING_SHOOTER,
        DISABLED
    }
    private IntakeTransferState currentState;
    private boolean isTelemetryEnabled = true;
    private OpMode opMode;

    public IntakeTransferSubsystem(OpMode opMode, IntakeTransferState intakeTransferState) {
        this.opMode = opMode;
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        feedMotor = opMode.hardwareMap.get(DcMotor.class, FEED_MOTOR_NAME);

        intakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        intakeMotor.setDirection(FEED_MOTOR_DIRECTION);
        this.currentState = intakeTransferState;
    }

    public void intake() {
        intakeMotor.setPower(ServoStates.SPINNING.servoPower);
        feedMotor.setPower(ServoStates.STOPPED.servoPower);
    }

    public void eject() {
        intakeMotor.setPower(ServoStates.REVERSED.servoPower);
        feedMotor.setPower(ServoStates.STOPPED.servoPower);
    }

    /**
     * Feeds a ball of a specific color into the shooter.
     */
    public void feedShooter() {
        intakeMotor.setPower(ServoStates.SPINNING.servoPower);
        feedMotor.setPower(Property.FEEDING_POWER);
    }



//--------------------Common functions across subsystems--------------------


    /**
     * gets called from robot container every loop
     */
    public void periodic() {
        switch (currentState) {
            case DISABLED:
                shutDownSubsystem();
                break;
            case INTAKING:
                intake();
                break;
            case EJECTING:
                eject();
                break;
            case FEEDING_SHOOTER:
                feedShooter();
                break;
        }
    }

    /**
     * Enables or disables telemetry output for this subsystem. defaults to true
     *
     * @param enabled True to enable telemetry, false to disable
     */
    @Override
    public void enableSubsystemTelemetry(boolean enabled) {
        isTelemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if(isTelemetryEnabled) {
            switch (currentState) {
                case DISABLED:
                    opMode.telemetry.addLine("Intake disabled");
                    break;
                case INTAKING:
                    opMode.telemetry.addLine("Intaking");
                    break;
                case EJECTING:
                    opMode.telemetry.addLine("Ejecting");
                case FEEDING_SHOOTER:
                    opMode.telemetry.addLine("Feeding Shooter");
            }
        }
    }

    /**
     * Resets the subsystem to a known safe state.
     */
    @Override
    public void resetSubsystem() {

    }

    /**
     * Safely shuts down the subsystem. Motors should stop
     */
    @Override
    public void shutDownSubsystem() {
        intakeMotor.setPower(ServoStates.STOPPED.servoPower);
        feedMotor.setPower(ServoStates.STOPPED.servoPower);
    }

    /**
     * Returns the current state of the subsystem.
     *
     * @return The current subsystem state (subsystem-specific enum)
     */
    @Override
    public IntakeTransferState getState() {
        return currentState;
    }
    public void setState(IntakeTransferState intakeTransferState) {
        currentState = intakeTransferState;
        switch (currentState) {
            case DISABLED:
                shutDownSubsystem();
                break;
            case INTAKING:
                intake();
                break;
            case EJECTING:
                eject();
                break;
            case FEEDING_SHOOTER:
                feedShooter();
                break;
        }
    }
}

