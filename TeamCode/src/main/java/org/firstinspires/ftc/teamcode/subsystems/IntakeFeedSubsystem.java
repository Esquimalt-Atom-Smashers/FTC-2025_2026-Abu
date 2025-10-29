package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeFeedSubsystem extends SubsystemBase {
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

    public IntakeFeedSubsystem(OpMode opMode) {
        IntakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        rearFeedServo = opMode.hardwareMap.get(CRServo.class, REAR_FEED_SERVO_NAME);
        feedServo = opMode.hardwareMap.get(CRServo.class, FEED_SERVO_NAME);

        IntakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        rearFeedServo.setDirection(REAR_FEED_SERVO_DIRECTION);
        feedServo.setDirection(FEED_SERVO_DIRECTION);
    }

    public void setIntakePower(ServoStates servoStates) {
        IntakeMotor.setPower(servoStates.servoPower);
    }

    public void setFeedPower(ServoStates servoStates) {
        feedServo.setPower(servoStates.servoPower);
        rearFeedServo.setPower(servoStates.servoPower);
    }

    public void stopArtifact() {
        feedServo.setPower(ServoStates.STOPPED.servoPower);
        rearFeedServo.setPower(ServoStates.REVERSED.servoPower);
    }
}
