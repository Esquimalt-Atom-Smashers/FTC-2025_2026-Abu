package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.opmode.FlywheelTuner;
@Config

public class IntakeFeedSubsystem extends SubsystemBase {
    private static DcMotor IntakeMotor;
    private static final String INTAKE_MOTOR_NAME = "intakeMotor";
    private static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    private static CRServo feedServo;
    private static final String FEED_SERVO_NAME = "feedServo";
    private static final DcMotorSimple.Direction FEED_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private static Servo rearFeedServo;
    private static final String REAR_FEED_SERVO_NAME = "rearFeedServo";
    private static final Servo.Direction REAR_FEED_SERVO_DIRECTION = Servo.Direction.FORWARD;

    private static CRServo spIndexServo;
    private static final String SP_INDEXER_NAME = "spIndexServo";
    private static final DcMotorSimple.Direction SP_INDEX_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static class Params {
        public double stopPower = -1.0;
        public double feedPower = 1.0;

        public double retractedPos = 0.49;
        public double extendedPos = 0.19;
    }
    public static Params PARAMS = new Params();

    public IntakeFeedSubsystem(OpMode opMode) {
        IntakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        rearFeedServo = opMode.hardwareMap.get(Servo.class, REAR_FEED_SERVO_NAME);
        feedServo = opMode.hardwareMap.get(CRServo.class, FEED_SERVO_NAME);
        spIndexServo = opMode.hardwareMap.get(CRServo.class, SP_INDEXER_NAME);

        IntakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        rearFeedServo.setDirection(REAR_FEED_SERVO_DIRECTION);
        feedServo.setDirection(FEED_SERVO_DIRECTION);
        spIndexServo.setDirection(SP_INDEX_SERVO_DIRECTION);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "initializeIntakeFeedSubsystem",
            parameterLabels = {}
    )
    public static void initializeIntakeFeedSubsystem() {
        IntakeMotor = opMode.hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        rearFeedServo = opMode.hardwareMap.get(Servo.class, REAR_FEED_SERVO_NAME);
        feedServo = opMode.hardwareMap.get(CRServo.class, FEED_SERVO_NAME);
        spIndexServo = opMode.hardwareMap.get(CRServo.class, SP_INDEXER_NAME);

        IntakeMotor.setDirection(INTAKE_MOTOR_DIRECTION);
        rearFeedServo.setDirection(REAR_FEED_SERVO_DIRECTION);
        feedServo.setDirection(FEED_SERVO_DIRECTION);
        spIndexServo.setDirection(SP_INDEX_SERVO_DIRECTION);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "setIntakePower",
            parameterLabels = {"servoPower"}
    )
    public static void setIntakePower(double servoPower) {
        IntakeMotor.setPower(servoPower);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "setFeedPower",
            parameterLabels = {"servoPower"}
    )
    public static void setFeedPower(double servoPower) {
        feedServo.setPower(servoPower);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "setRearFeedServoPos",
            parameterLabels = {"servoPos"}
    )
    public static void setRearFeedServoPos(double servoPos) {
        rearFeedServo.setPosition(servoPos);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "getRearFeedServoPos",
            parameterLabels = {}
    )
    public static double getRearFeedServoPos() {
        return rearFeedServo.getPosition();
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "setSpIndexServoPower",
            parameterLabels = {"servoPower"}
    )
    public static void setSpIndexServoPower(double power) {
        spIndexServo.setPower(power);
    }
}
