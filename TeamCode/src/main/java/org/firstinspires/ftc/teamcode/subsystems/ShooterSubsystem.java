package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

import java.util.TreeMap;

/**
 * ShooterSubsystem
 *
 * Controls the shooter mechanism, including flywheel speed
 * and hood angle adjustment.
 *
 * This class currently contains only structure and documentation.
 */
@Config
public class ShooterSubsystem implements SubsystemBase{
    public static class Params {
        public double TOLERANCE = 28;
        public double kS = 0;
        public double kV = Property.kV;
        public double kA = 0;

        public double P = 0;
        public double I = 0;
        public double D = 0;
    }
    public static Params PARAMS = new Params();
    private OpMode opMode;
    boolean isTelemetryEnabled = true;
    RobotContainer.Alliance alliance;

    private DcMotorEx flywheelMotor;
    private final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public final double RPM_TOLERANCE = Property.TOLERANCE;

    private final double TICKS_PER_ROTATION = 28;
    private FlywheelSetting targetSetting = new FlywheelSetting(0,0);
    //custom PID + feedforward
    private PIDController flyWheelController;
    private double maxFlywheelPower = 1.0;
    private Pose2d pose2d;

    //distance to rpm matching table, we are not using hood angle rn
    public static class FlywheelSetting {
        public double rpm;
        public double hoodAngle;

        public FlywheelSetting(double rpm, double hoodAngle) {
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
        }
    }

    private final TreeMap<Double, FlywheelSetting> MATCHING_MAP= new TreeMap<>();
    {
        MATCHING_MAP.put(30.0 , new FlywheelSetting(3300, 20));
        MATCHING_MAP.put(45.0, new FlywheelSetting(3050, 25));
        MATCHING_MAP.put(60.0, new FlywheelSetting(3100, 30));
        MATCHING_MAP.put(75.0, new FlywheelSetting(3100, 30));
        MATCHING_MAP.put(90.0, new FlywheelSetting(3150, 30));
        MATCHING_MAP.put(105.0, new FlywheelSetting(3350, 30));
        MATCHING_MAP.put(120.0, new FlywheelSetting(3550, 30));
        MATCHING_MAP.put(130.0, new FlywheelSetting(3600, 30));
    }

    public enum ShooterState{
        MANUAL,
        AUTO,
        DISABLED
    }
    private ShooterState currentState;
    /**
     * Constructor for the ShooterSubsystem.
     * Shooter motors, hood servos, and sensors
     * should be initialized here later.
     */
    public ShooterSubsystem(OpMode opMode, RobotContainer.Alliance alliance, ShooterState state, Pose2d pose2d) {
        this.opMode = opMode;
        this.alliance = alliance;
        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelController = new PIDController(PARAMS.P, PARAMS.I, PARAMS.D);
        currentState = state;
        setPose2d(pose2d);
        setFlywheelMotorPower(0);
    }

    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    public double rpmToTps(double rpm) {
        return rpm * TICKS_PER_ROTATION / 60;
    }

    public double getFlywheelRPM() {
        return -flywheelMotor.getVelocity() / TICKS_PER_ROTATION * 60;
    }

    public void setFlywheelMotorPower(double power) {
        flywheelMotor.setPower(-power);
    }

    public double flywheelCustomPID(double targetVelocity) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double pid = Range.clip(flyWheelController.calculate(flywheelMotor.getVelocity(), targetVelocity), -maxFlywheelPower, maxFlywheelPower);
        return pid;
    }

    public double flywheelFeedForward(double targetVelocity) {
        double feedForward = PARAMS.kS * Math.signum(targetVelocity) + PARAMS.kV * targetVelocity + PARAMS.kA * (getFlywheelRPM() - targetVelocity);
        return feedForward;
    }

    public void setPose2d(Pose2d pose2d){
        this.pose2d = pose2d;
    }

    /**
     * Shoots using a specific hood angle and flywheel RPM.
     * It is just a placeholder for setRPM, it will have its intended function later
     */
    public void shoot(FlywheelSetting targetSetting) {
        this.targetSetting = targetSetting;

        double targetVelocity = targetSetting.rpm;
        double feedforward = flywheelFeedForward(targetVelocity);
        double pid = flywheelCustomPID(targetVelocity);

        setFlywheelMotorPower(feedforward + pid);
    }

    /**
     * Shoots using preconfigured settings based on alliance color & increase feedforward.
     */
    public void shoot(RobotContainer.Alliance alliance, Pose2d robotPose) {
        Pose2d goalPos;
        if (alliance == RobotContainer.Alliance.RED) {
            goalPos = RobotContainer.RED_GOAL_POSE;
        } else {
            goalPos = RobotContainer.BLUE_GOAL_POSE;
        }
        double distance = Math.sqrt(Math.pow(robotPose.position.x - goalPos.position.x, 2) + Math.pow(robotPose.position.y - goalPos.position.y, 2));
        shoot(distanceToFlywheelSetting(distance));
    }

    //go through matching table
    //get distance in inch
    public FlywheelSetting distanceToFlywheelSetting(double distance) {
        FlywheelSetting lower;
        try { lower = MATCHING_MAP.floorEntry(distance).getValue();
        } catch (Exception e) {
            return MATCHING_MAP.ceilingEntry(distance).getValue();
        }
        FlywheelSetting higher;
        try { higher = MATCHING_MAP.ceilingEntry(distance).getValue();
        } catch (Exception e) {
            return MATCHING_MAP.floorEntry(distance).getValue();
        }
        if (lower == null) return higher;
        if (higher == null | lower == higher) return lower;

        double percentageDistance = (distance - MATCHING_MAP.floorKey(distance))
                / (MATCHING_MAP.ceilingKey(distance) - MATCHING_MAP.floorKey(distance));

        return new FlywheelSetting(lower.rpm + ((higher.rpm - lower.rpm) * percentageDistance),
                lower.hoodAngle + ((higher.hoodAngle - lower.hoodAngle) * percentageDistance));

    }
//--------------------Common functions across subsystems--------------------
    /**
     * Runs every loop
     */
    public void periodic() {
        if (currentState == ShooterState.DISABLED) {
            shutDownSubsystem();
        } else if (currentState == ShooterState.AUTO) {
            shoot(alliance, pose2d);
        } else {
            shoot(targetSetting);
        }
    }

    /**
     * Enables or disables telemetry output for this subsystem.
     *
     * @param enabled True to enable telemetry, false to disable
     */
    public void enableSubsystemTelemetry(boolean enabled) {
        isTelemetryEnabled = enabled;
    }

    @Override
    public void addSubsystemTelemetry() {
        if (isTelemetryEnabled) {
            opMode.telemetry.addData("RPM", "Target: %.2f, Current : %.2f", targetSetting.rpm, getFlywheelRPM());
        }
    }

    /**
     * Resets the subsystem to a known safe state.
     */
    public void resetSubsystem(){}

    /**
     * Safely shuts down the subsystem.
     *
     * Motors should stop and resources should be released.
     */
    public void shutDownSubsystem() {
        setFlywheelMotorPower(0);
    }

    public ShooterState getState() {
        return currentState;
    }
    public void setCurrentState(ShooterState shooterState) {currentState = shooterState;}

}
