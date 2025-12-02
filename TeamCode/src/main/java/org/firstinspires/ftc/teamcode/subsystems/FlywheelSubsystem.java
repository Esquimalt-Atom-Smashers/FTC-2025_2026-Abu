package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import java.util.TreeMap;

@Config
public class FlywheelSubsystem extends BlocksOpModeCompanion {
    public static class Params {
        public double TOLERANCE = 28;
        public double kS = 0;
        public double kV = 0.000475;
        public double kA = 0;

        public double P = 0.005;
        public double I = 0.001;
        public double D = 0;
    }
    public static Params PARAMS = new Params();
    private static OpMode opMode;

    private static DcMotorEx flywheelMotor;
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private static final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public final double RPM_TOLERANCE = 25.0;

    private static final double TICKS_PER_ROTATION = 28;
    private double targetVelocity; //ticks per second

    private final PIDFCoefficients FLYWHEEL_PIDF_SETTING = new PIDFCoefficients(200, 0, 0, 0);

    private static Servo hoodAngleServo;
    private static final String HOOD_ANGLE_SERVO_NAME = "hoodAngleServo";

    public final double FAR_SHOOTING_ANGLE = 0.0;
    public final double CLOSE_SHOOTING_ANGLE = 0.0;

    //custom PID + feedforward
    private static PIDController flyWheelController;
    private static double maxFlywheelPower = 1.0;

    //distance to rpm checking sheet
    public class FlywheelSetting {
        public double rpm;
        public double hoodAngle;

        public FlywheelSetting(double rpm, double hoodAngle) {
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
        }
    }

    private final TreeMap<Double, FlywheelSetting> MATCHING_MAP= new TreeMap<>();
    {
        MATCHING_MAP.put(1.0 , new FlywheelSetting(3000, 20));
        MATCHING_MAP.put(2.0, new FlywheelSetting(3500, 25));
        MATCHING_MAP.put(3.0, new FlywheelSetting(4000, 30));
    }

    public FlywheelSubsystem(OpMode opMode) {
        this.opMode = opMode;
        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelController = new PIDController(PARAMS.P, PARAMS.I, PARAMS.D);

        hoodAngleServo = opMode.hardwareMap.get(Servo.class, HOOD_ANGLE_SERVO_NAME);
        hoodAngleServo.setDirection(Servo.Direction.FORWARD);
    }

    @ExportToBlocks(
        comment = "",
        tooltip = "initializeFlywheelSubsystem",
        parameterLabels = {}
    )

    public static void initializeFlywheelSubsystem() {
        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelController = new PIDController(PARAMS.P, PARAMS.I, PARAMS.D);

        hoodAngleServo = opMode.hardwareMap.get(Servo.class, HOOD_ANGLE_SERVO_NAME);
        hoodAngleServo.setDirection(Servo.Direction.FORWARD);
    }

    public void runWithDefaultPID(double rpm){
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF_SETTING);
        targetVelocity = rpmToTps(rpm);
        flywheelMotor.setVelocity(targetVelocity);
    }

    //utilities
    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    public double rpmToTps(double rpm) {
        return rpm * TICKS_PER_ROTATION / 60;
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "initializeFlywheelSubsystem",
            parameterLabels = {}
    )
    public static double getFlywheelRPM() {
        return -flywheelMotor.getVelocity() / TICKS_PER_ROTATION * 60;
    }

    public double getTargetVelocity() {return targetVelocity;}

    //custom PID
    public void setFlywheelMaxPower(double maxPower) {
        this.maxFlywheelPower = maxPower;
    }

    public void setFlywheelTargetVelocity(double rpm) {
        targetVelocity = rpmToTps(-rpm);
    }

    public void updateFlywheelPID() {
        flyWheelController.setPID(PARAMS.P, PARAMS.I, PARAMS.D);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "setFlywheelMotorPower",
            parameterLabels = {"power"}
    )
    public static void setFlywheelMotorPower(double power) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        flywheelMotor.setPower(power);
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "flyWheelCustomPID",
            parameterLabels = {"targetVelocity"}
    )
    public static double flywheelCustomPID(double targetVelocity) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double pid = Range.clip(flyWheelController.calculate(flywheelMotor.getVelocity(), targetVelocity), -maxFlywheelPower, maxFlywheelPower);
        return pid;
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "flyWheelFeedForward",
            parameterLabels = {"targetVelocity"}
    )
    public static double flywheelFeedForward(double targetVelocity) {
        double feedForward = PARAMS.kS * Math.signum(targetVelocity) + PARAMS.kV * targetVelocity + PARAMS.kA * (getFlywheelRPM() - targetVelocity);
        return feedForward;
    }

    @ExportToBlocks(
            comment = "",
            tooltip = "setHoodAngle",
            parameterLabels = {"hoodAngle"}
    )
    public static void setHoodAngle(double hoodAngle) {
        hoodAngleServo.setPosition(hoodAngle);
    }

    //go through matching table
    //get distance in inch
    public FlywheelSetting distanceToFlywheelSetting(double distance) {
        FlywheelSetting lower = MATCHING_MAP.floorEntry(distance).getValue();
        FlywheelSetting higher = MATCHING_MAP.ceilingEntry(distance).getValue();

        double percentageDistance = (distance - MATCHING_MAP.floorKey(distance)) / (MATCHING_MAP.ceilingKey(distance) - MATCHING_MAP.floorKey(distance));
        if (!Double.isNaN(percentageDistance)) {
            return new FlywheelSetting(lower.rpm + ((higher.rpm - lower.rpm) * percentageDistance), lower.hoodAngle + ((higher.hoodAngle - lower.hoodAngle) * percentageDistance));
        } else {
            return lower;
        }
    }

    public void runFlywheelControl() {
        double feedforward = flywheelFeedForward(targetVelocity);
        double pid = flywheelCustomPID(targetVelocity);

        setFlywheelMotorPower(feedforward + pid);
    }

    public void runShooterControlByDistance(double distance) {
        FlywheelSetting flywheelSetting = distanceToFlywheelSetting(distance);
        setFlywheelTargetVelocity(flywheelSetting.rpm);
        runFlywheelControl();
    }
}
