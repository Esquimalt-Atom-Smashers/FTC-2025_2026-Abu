package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TwoMotorShooterSubsystem;
import org.firstinspires.ftc.teamcode.utilities.RobotContainer;

@TeleOp
@Config
public class TwoMotorFlywheelTuner extends OpMode {
    public DcMotor intakeMotor;
    public DcMotor feedMotor;
    public static class Params {
        public double stayTime = 10.0;
        public double accelerateTime = 0.0;
        public double maxRPM = 4000;
        public double minRPM = 3000;
        public double intakePower = 1;
        public double feedPower = 1;
        public double notFeedPower = 0;
        public double testSteps = 1;
        public double dPadUpDeg = 260;
        public double dPadDownDeg = 100;
        //max servo number 0.93
        //min servo num 0.27
    }

    public static Params PARAMS = new Params();

    private TwoMotorShooterSubsystem flywheelSubsystem;
    private TurretSubsystem turretSubsystem;
//    private IntakeTransferSubsystem intakeFeedSubsystem;

    private Servo hoodServo;

    // Hood tuning constants
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 1.0;
    private static final double HOOD_STEP = 0.01;

    private double hoodPosition = 0.5; // start mid-position

    private enum MotionProfilingStates {
        ACCLEARATING(PARAMS.maxRPM),
        CONSTANT(PARAMS.minRPM),
        DECELLERATING(PARAMS.minRPM);
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
    public double targetDegree;
    boolean prevUp = false;
    boolean prevDown = false;

    @Override
    public void init() {
//        intakeFeedSubsystem = new IntakeTransferSubsystem(this, IntakeTransferSubsystem.IntakeTransferState.INTAKING);
        turretSubsystem = new TurretSubsystem(this, RobotContainer.Alliance.RED, TurretSubsystem.TurretState.ROBOT_FRAME_LOCK, new Pose2d(0,0,0), new Pose2d(0,0,0));
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        feedMotor = hardwareMap.get(DcMotor.class, "feedMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo"); // port 0
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelSubsystem = new TwoMotorShooterSubsystem(this, RobotContainer.Alliance.RED, TwoMotorShooterSubsystem.ShooterState.MANUAL, new Pose2d(0, 0, 0));
        timer = new ElapsedTime();
        state = MotionProfilingStates.ACCLEARATING;
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        loopTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        // Manual controls for target and PID tuning
        if (gamepad1.x && !prevUp) {
            turretSubsystem.lockRobotFrame(turretSubsystem.getTurretAngleOnBot() + PARAMS.testSteps);
        }
        if (gamepad1.b && !prevDown) {
            turretSubsystem.lockRobotFrame(turretSubsystem.getTurretAngleOnBot() - PARAMS.testSteps);
        }
        prevUp = gamepad1.x;
        prevDown = gamepad1.b;
        if (gamepad1.dpad_up) {
            turretSubsystem.lockRobotFrame(PARAMS.dPadUpDeg);
        } else if (gamepad1.dpad_down) {
            turretSubsystem.lockRobotFrame(PARAMS.dPadDownDeg);
        }

        // Feed control
        if (gamepad1.a) {
            feedMotor.setPower(PARAMS.feedPower);
        } else {
            feedMotor.setPower(PARAMS.notFeedPower);
        }

        intakeMotor.setPower(PARAMS.intakePower);

        // Hood control (PORT 0)
        if (gamepad1.left_bumper) {
            hoodPosition += HOOD_STEP;
        } else if (gamepad1.right_bumper) {
            hoodPosition -= HOOD_STEP;
        }

        hoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPosition));
        hoodServo.setPosition(hoodPosition);

        // Shooter control
        double targetRPM = motionProfiling();
        flywheelSubsystem.shoot(
                new TwoMotorShooterSubsystem.FlywheelSetting(targetRPM, hoodPosition)
        );
        turretSubsystem.updateRobotPose(new Pose2d(0,0,0), new PoseVelocity2d(new Vector2d(0,0),0));
        turretSubsystem.periodic();

        // Telemetry
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("currentRPM", flywheelSubsystem.getFlywheelRPM());
        telemetry.addData("hood position", hoodPosition);
        turretSubsystem.addSubsystemTelemetry();
        telemetry.addData("latency (ms)", loopTimer.milliseconds());
        turretSubsystem.addAxonLog();

        loopTimer.reset();
        telemetry.update();
    }

    private double motionProfiling() {
        double targetRPM;
        if (PARAMS.maxRPM == PARAMS.minRPM) {
            targetRPM = PARAMS.maxRPM;
        } else {
            if (timer.seconds() >= PARAMS.stayTime && state == MotionProfilingStates.ACCLEARATING) {
                pastState = state;
                state = MotionProfilingStates.CONSTANT;
                timer.reset();
            } else if (timer.seconds() >= PARAMS.accelerateTime && state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.ACCLEARATING) {
                pastState = state;
                state = MotionProfilingStates.DECELLERATING;
                timer.reset();
            } else if (timer.seconds() >= PARAMS.accelerateTime && state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.DECELLERATING) {
                pastState = state;
                state = MotionProfilingStates.ACCLEARATING;
                timer.reset();
            } else if (timer.seconds() >= PARAMS.stayTime && state == MotionProfilingStates.DECELLERATING) {
                pastState = state;
                state = MotionProfilingStates.CONSTANT;
                timer.reset();
            }

            if (state == MotionProfilingStates.ACCLEARATING) {
                if (PARAMS.accelerateTime == 0.0) {
                    targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM();
                } else {
                    targetRPM = MotionProfilingStates.CONSTANT.getTargetRPM() + (timer.seconds() / PARAMS.accelerateTime) * (MotionProfilingStates.ACCLEARATING.getTargetRPM() - MotionProfilingStates.CONSTANT.getTargetRPM());
                }
            } else if (state == MotionProfilingStates.DECELLERATING) {

                if (PARAMS.accelerateTime == 0.0) {
                    targetRPM = MotionProfilingStates.DECELLERATING.getTargetRPM();
                } else {
                    targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM() - (timer.seconds() / PARAMS.accelerateTime) * (MotionProfilingStates.ACCLEARATING.getTargetRPM() - MotionProfilingStates.CONSTANT.getTargetRPM());
                }
            } else if (state == MotionProfilingStates.CONSTANT && pastState == MotionProfilingStates.ACCLEARATING) {
                targetRPM = MotionProfilingStates.ACCLEARATING.getTargetRPM();
            } else {
                targetRPM = MotionProfilingStates.CONSTANT.getTargetRPM();
            }
        }
        return targetRPM;
    }
}

/**
 * ShooterSubsystem
 *
 * Controls the shooter mechanism, including flywheel speed
 * and hood angle adjustment.
 *
 * This class currently contains only structure and documentation.
 */
