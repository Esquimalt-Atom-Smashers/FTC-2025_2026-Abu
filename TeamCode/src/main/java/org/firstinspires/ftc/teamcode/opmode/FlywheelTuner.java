package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeedSubsystem;
@TeleOp
@Config
public class FlywheelTuner extends OpMode {
    public static class Params {
        public double stayTime = 10.0;
        public double upperRPM = 4300;
        public double lowerRPM = 4000;

    }
    public static Params PARAMS = new Params();

    private FlywheelSubsystem flywheelSubsystem;
    private IntakeFeedSubsystem intakeFeedSubsystem;
    private enum MotionProfilingStates{
        ACCLEARATING(PARAMS.upperRPM),
        CONSTANT(PARAMS.lowerRPM),
        DECELLERATING(PARAMS.lowerRPM);
        public double targetRPM;
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

    @Override
    public void init() {
        intakeFeedSubsystem = new IntakeFeedSubsystem(this);
        flywheelSubsystem = new FlywheelSubsystem(this);
        timer = new ElapsedTime();
        MotionProfilingStates.ACCLEARATING.targetRPM = PARAMS.upperRPM;
        MotionProfilingStates.CONSTANT.targetRPM = PARAMS.lowerRPM;
        MotionProfilingStates.DECELLERATING.targetRPM = PARAMS.lowerRPM;

        state = MotionProfilingStates.ACCLEARATING;
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intakeFeedSubsystem.setFeedPower(intakeFeedSubsystem.PARAMS.feedPower);
        } else {
            intakeFeedSubsystem.setFeedPower(intakeFeedSubsystem.PARAMS.stopPower);
        }

        if (gamepad1.b) {
            intakeFeedSubsystem.setSpIndexServoPower(1.0);
        } else {
            intakeFeedSubsystem.setSpIndexServoPower(0.0);
        }

        if (gamepad1.x) {
            flywheelSubsystem.setHoodAngle(flywheelSubsystem.PARAMS.CLOSE_SHOOTING_ANGLE);
        }
        if (gamepad1.y) {
            flywheelSubsystem.setHoodAngle(flywheelSubsystem.PARAMS.FAR_SHOOTING_ANGLE);
        }

        if (gamepad1.left_trigger >= 0.1) {
            intakeFeedSubsystem.setRearFeedServoPos(intakeFeedSubsystem.PARAMS.extendedPos);
        } else {
            intakeFeedSubsystem.setRearFeedServoPos(intakeFeedSubsystem.PARAMS.retractedPos);
        }

        double targetRPM = -motionProfiling();
        flywheelSubsystem.setFlywheelTargetVelocity(targetRPM);
        flywheelSubsystem.updateFlywheelPID();
        flywheelSubsystem.runFlywheelControl();
        telemetry.addData("targetRPM", -targetRPM);
        telemetry.addData("currentRPM", -flywheelSubsystem.getFlywheelRPM());
        telemetry.addData("output power", (flywheelSubsystem.getFlywheelPower() * -10000));
        telemetry.addData("hoodAngle", flywheelSubsystem.getHoodAngle());
        telemetry.addData("rearFeedPos", intakeFeedSubsystem.getRearFeedServoPos());
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
