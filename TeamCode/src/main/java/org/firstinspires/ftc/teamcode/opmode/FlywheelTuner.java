package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
@TeleOp
public class FlywheelTuner extends OpMode {
    private FlywheelSubsystem flywheelSubsystem;
    private enum MotionProfilingStates{
        ACCLEARATING(3400),
        CONSTANT(2000),
        DECELLERATING(2000);

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

    @Override
    public void init() {
        flywheelSubsystem = new FlywheelSubsystem(this);
        timer = new ElapsedTime();
        state = MotionProfilingStates.ACCLEARATING;
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        double targetRPM = motionProfiling();
        flywheelSubsystem.setFlywheelTargetVelocity(targetRPM);
        flywheelSubsystem.runFlywheelControl();
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("currentRPM", flywheelSubsystem.getFlywheelRPM());
        telemetry.update();
    }

    private double motionProfiling() {
        double targetRPM;
        if (timer.seconds() >= 5 && state == MotionProfilingStates.ACCLEARATING) {
            pastState = state;
            state = MotionProfilingStates.CONSTANT;
            timer.reset();
        } else if (timer.seconds() >= 2 && state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.ACCLEARATING) {
            pastState = state;
            state = MotionProfilingStates.DECELLERATING;
            timer.reset();
        } else if (timer.seconds() >= 2 && state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.DECELLERATING) {
            pastState = state;
            state = MotionProfilingStates.ACCLEARATING;
            timer.reset();
        } else if (timer.seconds() >= 5 && state == MotionProfilingStates.DECELLERATING) {
            pastState = state;
            state = MotionProfilingStates.CONSTANT;
            timer.reset();
        }

        if (state == MotionProfilingStates.ACCLEARATING) {
            targetRPM = MotionProfilingStates.CONSTANT.getTargetRPM() + (timer.seconds() / 5.0) * (MotionProfilingStates.ACCLEARATING.getTargetRPM() - MotionProfilingStates.CONSTANT.getTargetRPM());
        } else if (state == MotionProfilingStates.DECELLERATING) {
            targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM() - (timer.seconds() / 5.0) * (MotionProfilingStates.ACCLEARATING.getTargetRPM() - MotionProfilingStates.CONSTANT.getTargetRPM());
        } else if (state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.ACCLEARATING){
            targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM();
        } else {
            targetRPM = MotionProfilingStates.CONSTANT.getTargetRPM();
        }
        return targetRPM;
    }
}
