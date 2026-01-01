package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    private DcMotor IntakeMotor;
    private final String INTAKE_MOTOR_NAME = "intakeMotor";
    private final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private CRServo feedServo;
    private final String FEED_SERVO_NAME = "feedServo";
    private final DcMotorSimple.Direction FEED_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private CRServo rearFeedServo;
    private final String REAR_FEED_SERVO_NAME = "rearFeedServo";
    private final DcMotorSimple.Direction REAR_FEED_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;

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
        IntakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        rearFeedServo = opMode.hardwareMap.get(CRServo.class, REAR_FEED_SERVO_NAME);
        feedServo = opMode.hardwareMap.get(CRServo.class, FEED_SERVO_NAME);

        IntakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        rearFeedServo.setDirection(REAR_FEED_SERVO_DIRECTION);
        feedServo.setDirection(FEED_SERVO_DIRECTION);
        this.currentState = intakeTransferState;
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
     * Runs the intake to collect or move balls.
     *
     * @param ballColour The color of ball to intake
     * @param numberOfBalls The number of balls to intake
     * @param direction The direction to run the intake
     */
    public void intake(BallColour ballColour, int numberOfBalls, Direction direction) {
        // TODO: Implement intake logic
    }

    public void intake() {
        IntakeMotor.setPower(ServoStates.SPINNING.servoPower);
        feedServo.setPower(ServoStates.SPINNING.servoPower);
        rearFeedServo.setPower(ServoStates.REVERSED.servoPower);
    }

    /**
     * Ejects a specified number of balls of a given color.
     *
     * @param numberOfBalls The number of balls to eject
     * @param ballColour The color of balls to eject
     */
    public void eject(int numberOfBalls, BallColour ballColour) {
    }

    public void eject() {
        IntakeMotor.setPower(ServoStates.REVERSED.servoPower);
        feedServo.setPower(ServoStates.REVERSED.servoPower);
        rearFeedServo.setPower(ServoStates.REVERSED.servoPower);
    }

    /**
     * Feeds a ball of a specific color into the shooter.
     */
    public void feedShooter() {
        IntakeMotor.setPower(ServoStates.SPINNING.servoPower);
        feedServo.setPower(ServoStates.SPINNING.servoPower);
        rearFeedServo.setPower(ServoStates.SPINNING.servoPower);
    }



//--------------------Common functions across subsystems--------------------


    /**
     * gets called from robot container every loop
     */
    @Override
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
        IntakeMotor.setPower(ServoStates.STOPPED.servoPower);
        feedServo.setPower(ServoStates.STOPPED.servoPower);
        rearFeedServo.setPower(ServoStates.STOPPED.servoPower);
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

