package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

@TeleOp
@Config
public class FlywheelTuner extends OpMode {
    public static class Params {
        public double stayTime = 10.0;
    }
    public static Params PARAMS = new Params();

    private ShooterSubsystem flywheelSubsystem;
    private IntakeTransferSubsystem intakeFeedSubsystem;
    private enum MotionProfilingStates{
        ACCLEARATING(4000),
        CONSTANT(3000),
        DECELLERATING(3000);
        private double targetRPM;
        private MotionProfilingStates(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return targetRPM;
        }
    }
    private MotionProfilingStates state;
    private MotionProfilingStates pastState;
    private ElapsedTime timer;
    private ElapsedTime loopTimer;

    @Override
    public void init() {
        intakeFeedSubsystem = new IntakeTransferSubsystem(this, IntakeTransferSubsystem.IntakeTransferState.INTAKING);
        flywheelSubsystem = new ShooterSubsystem(this, RobotContainer.Alliance.RED, ShooterSubsystem.ShooterState.MANUAL, new Pose2d(0,0,0));
        timer = new ElapsedTime();
        state = MotionProfilingStates.ACCLEARATING;
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        loopTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intakeFeedSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.FEEDING_SHOOTER);
        } else {
            intakeFeedSubsystem.setState(IntakeTransferSubsystem.IntakeTransferState.INTAKING);
        }

        double targetRPM = -motionProfiling();
        flywheelSubsystem.shoot(new ShooterSubsystem.FlywheelSetting(targetRPM, 0));
        intakeFeedSubsystem.periodic();
        telemetry.addData("targetRPM", -targetRPM);
        telemetry.addData("currentRPM", -flywheelSubsystem.getFlywheelRPM());
        telemetry.addData("output power", (flywheelSubsystem.getFlywheelPower() * -10000));
        telemetry.addData("latency time", loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();
    }

    private double motionProfiling() {
        double targetRPM;
        if (timer.seconds() >= PARAMS.stayTime && state == MotionProfilingStates.ACCLEARATING) {
            pastState = state;
            state = MotionProfilingStates.CONSTANT;
            timer.reset();
        } else if (timer.seconds() >= PARAMS.stayTime && state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.ACCLEARATING) {
            pastState = state;
            state = MotionProfilingStates.DECELLERATING;
            timer.reset();
        } else if (timer.seconds() >= PARAMS.stayTime && state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.DECELLERATING) {
            pastState = state;
            state = MotionProfilingStates.ACCLEARATING;
            timer.reset();
        } else if (timer.seconds() >= PARAMS.stayTime && state == MotionProfilingStates.DECELLERATING) {
            pastState = state;
            state = MotionProfilingStates.CONSTANT;
            timer.reset();
        }

        if (state == MotionProfilingStates.ACCLEARATING) {
            targetRPM = MotionProfilingStates.CONSTANT.getTargetRPM() + (timer.seconds() / PARAMS.stayTime) * (MotionProfilingStates.ACCLEARATING.getTargetRPM() - MotionProfilingStates.CONSTANT.getTargetRPM());
        } else if (state == MotionProfilingStates.DECELLERATING) {
            targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM() - (timer.seconds() / PARAMS.stayTime) * (MotionProfilingStates.ACCLEARATING.getTargetRPM() - MotionProfilingStates.CONSTANT.getTargetRPM());
        } else if (state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.ACCLEARATING){
            targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM();
        } else {
            targetRPM = MotionProfilingStates.CONSTANT.getTargetRPM();
        }
        return targetRPM;
    }
}