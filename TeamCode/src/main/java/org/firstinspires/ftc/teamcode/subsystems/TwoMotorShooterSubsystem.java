package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opensource.FTC.RTPAxon.RTPAxon.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.utilities.Property.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opensource.FTC.RTPAxon.RTPAxon;
import org.firstinspires.ftc.teamcode.utilities.Property;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

import java.util.TreeMap;

@Config
public class TwoMotorShooterSubsystem implements SubsystemBase {
    public static class Params {
        public double additionalFirePower = 0.0;
        public double kS = 0.07;
        public double kV = 0.000165;
        public double nominalVoltage = 13.0;

        public double P = 0.007;
        public double I = 0;
        public double D = 0.0000001;
    }
    public static Params PARAMS = new Params();
    private OpMode opMode;
    boolean isTelemetryEnabled = true;
    RobotContainer.Alliance alliance;

    private DcMotorEx flywheelMotor;
    private DcMotorEx secondFlywheelMotor;
    private final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private final String SECOND_FLYWHEEL_MOTOR_NAME = "secondFlywheelMotor";
    private final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private final DcMotorSimple.Direction SECOND_FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public final double RPM_TOLERANCE = Property.TOLERANCE;
    private final String HOOD_SERVO_NAME = "hoodServo";
    private Servo hoodAngleServo;
    private VoltageSensor ExpansionHub2_VoltageSensor;

    private final double TICKS_PER_ROTATION = 28;
    private FlywheelSetting targetSetting = new FlywheelSetting(0,0);
    //custom PID + feedforward
    private double maxFlywheelPower = 1.0;
    private Pose2d pose2d;
    private double flywheelPower = 0.0;
    private double cachedVoltage;
    private ElapsedTime voltageTimer;

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
        MATCHING_MAP.put(30.0 , new FlywheelSetting(Property.THIRTY_INCH_RPM, THIRTY_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(45.0, new FlywheelSetting(Property.FORTY_FIVE_INCH_RPM, FORTY_FIVE_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(60.0, new FlywheelSetting(Property.SIXTY_INCH_RPM, SIXTY_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(75.0, new FlywheelSetting(Property.SEVENTY_FIVE_INCH_RPM, SEVENTY_FIVE_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(90.0, new FlywheelSetting(Property.NINETY_INCH_RPM, NINETY_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(105.0, new FlywheelSetting(Property.ONE_HUNDRED_FIVE_INCH_RPM, ONE_HUNDRED_FIVE_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(120.0, new FlywheelSetting(Property.ONE_HUNDRED_TWENTY_INCH_RPM, ONE_HUNDRED_TWENTY_INCH_HOOD_ANGLE));
        MATCHING_MAP.put(130.0, new FlywheelSetting(Property.ONE_HUNDRED_THIRTY_INCH_RPM, ONE_HUNDRED_THIRTY_INCH_HOOD_ANGLE));
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

    public TwoMotorShooterSubsystem(OpMode opMode, RobotContainer.Alliance alliance, ShooterState state, Pose2d pose2d) {
        this.opMode = opMode;
        this.alliance = alliance;
        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        secondFlywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, SECOND_FLYWHEEL_MOTOR_NAME);
        secondFlywheelMotor.setDirection(SECOND_FLYWHEEL_DIRECTION);
        secondFlywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        secondFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ExpansionHub2_VoltageSensor = opMode.hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        currentState = state;
        setPose2d(pose2d);
        setFlywheelMotorPower(0);

        hoodAngleServo = opMode.hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        hoodAngleServo.setDirection(Servo.Direction.FORWARD); // or REVERSE if needed
        voltageTimer = new ElapsedTime();
        voltageTimer.reset();
    }

    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    public double rpmToTps(double rpm) {
        return rpm * TICKS_PER_ROTATION / 60;
    }

    public double getFlywheelRPM() {
        return flywheelMotor.getVelocity() / TICKS_PER_ROTATION * 60;
    }

    public void setFlywheelMotorPower(double power) {
        flywheelPower = power;
        flywheelMotor.setPower(-power);
        secondFlywheelMotor.setPower(-power);
    }

    public double flywheelCustomPID(double targetVelocity) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double pid = Range.clip((targetVelocity - getFlywheelRPM()) * PARAMS.P, -maxFlywheelPower, maxFlywheelPower);
        return pid;
    }

    public double flywheelFeedForward(double targetVelocity, boolean isFeeding) {
        double ff =
                PARAMS.kS * Math.signum(targetVelocity) +
                        PARAMS.kV * targetVelocity +
                        (isFeeding? PARAMS.additionalFirePower: 0);

        if(voltageTimer.milliseconds() > 100){
            cachedVoltage = ExpansionHub2_VoltageSensor.getVoltage();
            voltageTimer.reset();
        }


        return ff * (PARAMS.nominalVoltage / cachedVoltage);
    }

    public void setPose2d(Pose2d pose2d){
        this.pose2d = pose2d;
    }

    /**
     * Shoots using a specific hood angle and flywheel RPM.
     * It is just a placeholder for setRPM, it will have its intended function later
     */
    public void shoot(FlywheelSetting targetSetting, boolean isFeeding) {
        this.targetSetting = targetSetting;

        double targetVelocity = targetSetting.rpm;
        double feedforward = flywheelFeedForward(targetVelocity, isFeeding);
        double pid = flywheelCustomPID(targetVelocity);

        hoodAngleServo.setPosition(targetSetting.hoodAngle);
        flywheelPower = feedforward + pid;
        setFlywheelMotorPower(flywheelPower);
    }

    public void setTargetSetting (FlywheelSetting targetSetting) {
        this.targetSetting = targetSetting;
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
    public void periodic(boolean isFeeding) {
        if (currentState == ShooterState.DISABLED) {
            shutDownSubsystem();
        } else {
            shoot(targetSetting, isFeeding);
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
            opMode.telemetry.addData("HoodAngle", hoodAngleServo.getPosition());
            opMode.telemetry.addData("FW Power", flywheelPower);
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

