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
import com.qualcomm.robotcore.util.Range;

@Config
public class FlywheelSubsystem extends SubsystemBase {
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
    private OpMode opMode;

    private DcMotorEx flywheelMotor;
    private final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private final DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;

    private final double WHEEL_RADIUS = 4; //inch
    private final double TICKS_PER_ROTATION = 28;
    private double targetVelocity; //ticks per second
    private double currentVelocity;

    private final PIDFCoefficients FLYWHEEL_PIDF_SETTING = new PIDFCoefficients(200, 0, 0, 0);

    //custom PID + feedforward
    private PIDController flyWheelController;
    private double maxFlywheelPower = 1.0;


    public FlywheelSubsystem(OpMode opMode) {
        this.opMode = opMode;
        flywheelMotor = opMode.hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelController = new PIDController(PARAMS.P, PARAMS.I, PARAMS.D);
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

    public double getFlywheelRPM() {
        return flywheelMotor.getVelocity() / TICKS_PER_ROTATION * 60;
    }

    //custom PID
    public void setFlywheelMaxPower(double maxPower) {
        this.maxFlywheelPower = maxPower;
    }

    public void setFlywheelTargetVelocity(double rpm) {
        targetVelocity = rpmToTps(rpm);
    }

    public void updateFlywheelPID() {
        flyWheelController.setPID(PARAMS.P, PARAMS.I, PARAMS.D);
    }

    public void setFlywheelMotorPower(double power) {
        if (flywheelMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        flywheelMotor.setPower(power);
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

    public void runFlywheelControl() {
        double feedforward = flywheelFeedForward(targetVelocity);
        double pid = flywheelCustomPID(targetVelocity);

        setFlywheelMotorPower(feedforward + pid);
    }

    public class RunFlywheelAction implements Action {

        public RunFlywheelAction(double targetRPM, double maxPower) {
            targetVelocity = rpmToTps(targetRPM);
            maxFlywheelPower = maxPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runFlywheelControl();
            return true;
        }
    }
}
