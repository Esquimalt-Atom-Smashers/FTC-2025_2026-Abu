package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeFeedSubsystem extends SubsystemBase {
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private final String LEFT_INTAKE_SERVO_NAME = "leftIntakeServo";
    private final String RIGHT_INTAKE_SERVO_NAME = "rightIntakeServo";
    private final DcMotorSimple.Direction LEFT_INTAKE_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction RIGHT_INTAKE_SERVO_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private CRServo feedServo;
    private final String FEED_SERVO_NAME = "feedServo";
    private final DcMotorSimple.Direction FEED_SERVO_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public enum ServoStates {
        SPINNING(1.0),
        STOPPED(0.0);

        private double servoPower;
        private ServoStates(double servoPower) {
            this.servoPower = servoPower;
        }
    }

    public IntakeFeedSubsystem(OpMode opMode) {
        leftIntakeServo = opMode.hardwareMap.get(CRServo.class, LEFT_INTAKE_SERVO_NAME);
        rightIntakeServo = opMode.hardwareMap.get(CRServo.class, RIGHT_INTAKE_SERVO_NAME);
        feedServo = opMode.hardwareMap.get(CRServo.class, FEED_SERVO_NAME);

        leftIntakeServo.setDirection(LEFT_INTAKE_SERVO_DIRECTION);
        rightIntakeServo.setDirection(RIGHT_INTAKE_SERVO_DIRECTION);
        feedServo.setDirection(FEED_SERVO_DIRECTION);
    }

    public void setIntakePower(ServoStates servoStates) {
        leftIntakeServo.setPower(servoStates.servoPower);
        rightIntakeServo.setPower(servoStates.servoPower);
    }

    public void setFeedPower(ServoStates servoStates) {
        feedServo.setPower(servoStates.servoPower);
    }
}
